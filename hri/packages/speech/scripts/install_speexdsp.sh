#!/bin/bash

# Installation script for SpeexDSP on macOS and Linux

echo "Installing SpeexDSP..."

# Check if we're on macOS or Linux
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    echo "Detected macOS"
    
    # Check if Homebrew is installed
    if ! command -v brew &> /dev/null; then
        echo "Homebrew not found. Please install Homebrew first:"
        echo "/bin/bash -c \"$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
        exit 1
    fi
    
    # Install speexdsp using Homebrew
    brew install speexdsp
    
    # Check installation
    if [ -f "/opt/homebrew/lib/libspeexdsp.dylib" ] || [ -f "/usr/local/lib/libspeexdsp.dylib" ]; then
        echo "SpeexDSP successfully installed!"
    else
        echo "SpeexDSP installation failed!"
        exit 1
    fi
    
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux
    echo "Detected Linux"
    
    # Check if it's Ubuntu/Debian
    if command -v apt-get &> /dev/null; then
        sudo apt-get update
        sudo apt-get install -y libspeexdsp-dev
    # Check if it's Red Hat/CentOS/Fedora
    elif command -v yum &> /dev/null; then
        sudo yum install -y speexdsp-devel
    elif command -v dnf &> /dev/null; then
        sudo dnf install -y speexdsp-devel
    else
        echo "Unsupported Linux distribution. Please install libspeexdsp manually."
        exit 1
    fi
    
    # Check installation
    if [ -f "/usr/lib/libspeexdsp.so" ] || [ -f "/usr/local/lib/libspeexdsp.so" ] || [ -f "/usr/lib/x86_64-linux-gnu/libspeexdsp.so" ]; then
        echo "SpeexDSP successfully installed!"
    else
        echo "SpeexDSP installation failed!"
        exit 1
    fi
    
else
    echo "Unsupported operating system: $OSTYPE"
    echo "Please install SpeexDSP manually for your platform."
    exit 1
fi

echo "SpeexDSP installation completed!"
