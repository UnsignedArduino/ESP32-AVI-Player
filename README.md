# ESP32-AVI-Player

An AVI video player for the M5Stack Core2.

## Install

1. Ensure you have PlatformIO installed.
2. Clone this repo.

## Build and upload

1. `pio run --target upload` or open in VS Code with the PlatformIO extension and run `PlatformIO: m5stack-core2 > Project tasks > General > Upload`

## FFmpeg command

We use Cinepak as the video codec and MP3 as the audio codec.

```cmd
ffmpeg -i INPUT.mp4 -c:a mp3 -c:v cinepak -q:v 10 -vf "fps=15,scale=iw*min(1\,if(gt(iw\,ih)\,320/iw\,(240*sar)/ih)):(floor((ow/dar)/4))*4:flags=lanczos" OUTPUT.avi
```
