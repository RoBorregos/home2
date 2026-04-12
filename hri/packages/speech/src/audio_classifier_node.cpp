#include <rclcpp/rclcpp.hpp>
#include <frida_interfaces/msg/audio_data.hpp>
#include "ei_run_classifier.h"

#include <vector>
#include <algorithm>
#include <cstdint>

class AudioClassifierNode : public rclcpp::Node
{
public:
    AudioClassifierNode() : Node("audio_classifier_node")
    {
        const std::string raw_audio_topic = "/hri/rawAudioChunk";

        stride_ = 8000; // 500ms sliding window stride

        subscription_ = this->create_subscription<frida_interfaces::msg::AudioData>(
            raw_audio_topic, 10,
            std::bind(&AudioClassifierNode::audio_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Audio Classifier Node initiated, listening to: %s", raw_audio_topic.c_str());
    }

private:
    void audio_callback(const frida_interfaces::msg::AudioData::SharedPtr msg)
    {
        size_t incoming_bytes = msg->data.size();
        size_t incoming_samples = incoming_bytes / 2;

        for (size_t i = 0; i < incoming_samples; i++) {
            // Reconstruct int16_t from little-endian bytes to avoid memory alignment issues
            int16_t sample = static_cast<int16_t>(
                msg->data[i * 2] | (msg->data[i * 2 + 1] << 8)
            );
            audio_buffer_.push_back(static_cast<float>(sample));
        }

        while (audio_buffer_.size() >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            
            float max_amp = 0.0f;
            for (size_t k = 0; k < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; k++) {
                if (std::abs(audio_buffer_[k]) > max_amp) {
                    max_amp = std::abs(audio_buffer_[k]);
                }
            }

            signal_t signal;
            int err = numpy::signal_from_buffer(audio_buffer_.data(), EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

            if (err == 0) {
                ei_impulse_result_t result = { 0 };
                EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

                if (res == EI_IMPULSE_OK) {
                    std::string best_label = "none";
                    float best_conf = 0.0f;

                    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                        std::string label = result.classification[ix].label;
                        float confidence = result.classification[ix].value;
                        
                        if (confidence > best_conf) {
                            best_conf = confidence;
                            best_label = label;
                        }

                        if (confidence > 0.65f && (label == "knock" || label == "doorBell")) {
                            RCLCPP_INFO(this->get_logger(), "DETECTED: [%s] (confidence: %.2f) | Max Vol: %.0f", label.c_str(), confidence, max_amp);
                        }
                    }
                    
                    RCLCPP_DEBUG(this->get_logger(), "Top pred: [%s] (%.2f) | Max Vol: %.0f", best_label.c_str(), best_conf, max_amp);
                }
            }

            // Shift the sliding window left by the stride amount
            audio_buffer_.erase(audio_buffer_.begin(), audio_buffer_.begin() + stride_);
        }
    }

    rclcpp::Subscription<frida_interfaces::msg::AudioData>::SharedPtr subscription_;
    std::vector<float> audio_buffer_;
    size_t stride_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioClassifierNode>());
    rclcpp::shutdown();
    return 0;
}