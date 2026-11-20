# Regenerates the simulated guest's answers (16 kHz mono int16, what the STT expects) with the
# offline Windows voices. Run on Windows: powershell -ExecutionPolicy Bypass -File generate.ps1
# Clip names are what sim_guest.py's pick_answer() returns.
Add-Type -AssemblyName System.Speech
$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
$lines = [ordered]@{
  'g1_name'  = 'My name is Charlie.'
  'g1_drink' = 'My favorite drink is lemonade.'
  'g2_name'  = 'Hi, I am Jane.'
  'g2_drink' = 'I would like some milk please.'
  'yes'      = 'Yes.'
  'no'       = 'No.'
  'dest'     = 'Please go to the kitchen.'
  'stop'     = 'Stop.'
}
foreach ($k in $lines.Keys) {
  $voice = if ($k -like 'g2*') { 'Microsoft Zira Desktop' } else { 'Microsoft David Desktop' }
  $s = New-Object System.Speech.Synthesis.SpeechSynthesizer
  $s.SelectVoice($voice); $s.Rate = -1
  $s.SetOutputToWaveFile((Join-Path $PSScriptRoot "$k.wav"), $fmt); $s.Speak($lines[$k]); $s.Dispose()
}
