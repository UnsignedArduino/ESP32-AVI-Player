#include <Arduino.h>
#include <M5Core2.h>
#include <WiFi.h>
#include <avilib.h>

static avi_t* a;
static long frames, estimateBufferSize, aRate, aBytes, aChunks, actual_video_size;
static int w, h, aChans, aBits, aFormat;
static double fr;
static char* compressor;
static char* vidbuf;
static char* audbuf;
static bool isStopped = true;
static int curr_frame = 0;
static unsigned long start_ms;

const char* avi_file = "/video.avi";

void setup() {
  M5.begin();
  WiFi.mode(WIFI_OFF);

  Serial.println("avilib");

  a = AVI_open_input_file(avi_file, 1);

  frames = AVI_video_frames(a);
  w = AVI_video_width(a);
  h = AVI_video_height(a);
  fr = AVI_frame_rate(a);
  compressor = AVI_video_compressor(a);
  estimateBufferSize = w * h * 4 / 7;
  Serial.printf("AVI frames: %d, %d x %d @ %.2f fps, format: %s, estimateBufferSize: %d\n", frames, w, h, fr,
                compressor, estimateBufferSize);

  aChans = AVI_audio_channels(a);
  aBits = AVI_audio_bits(a);
  aFormat = AVI_audio_format(a);
  aRate = AVI_audio_rate(a);
  aBytes = AVI_audio_bytes(a);
  aChunks = AVI_audio_chunks(a);
  Serial.printf("Audio channels: %d, bits: %d, format: %d, rate: %d, bytes: %d, chunks: %d\n", aChans, aBits, aFormat,
                aRate, aBytes, aChunks);

  vidbuf = (char*)malloc(estimateBufferSize);
  audbuf = (char*)malloc(1024);

  isStopped = false;
  start_ms = millis();
}

void loop() {
  if (!isStopped) {
    if (curr_frame < frames) {
      AVI_set_video_position(a, curr_frame);

      long audio_bytes = AVI_audio_size(a, curr_frame);
      AVI_read_audio(a, audbuf, audio_bytes);

      int iskeyframe;
      long video_bytes = AVI_frame_size(a, curr_frame);
      if (video_bytes > estimateBufferSize) {
        Serial.printf("%d/%d: video_bytes(%d) > estimateBufferSize(%d)\n", curr_frame, frames, video_bytes,
                      estimateBufferSize);
      } else {
        actual_video_size = AVI_read_frame(a, vidbuf, &iskeyframe);
        // Serial.printf("%d/%d: video_bytes(%d)\n", curr_frame, frames, actual_video_size);
      }

      // Serial.printf("frame: %d, iskeyframe: %d, video_bytes: %d, actual_video_size: %d, audio_bytes: %d\n",
      // curr_frame, iskeyframe, video_bytes, actual_video_size, audio_bytes);

      curr_frame++;
    } else {
      AVI_close(a);
      isStopped = true;
      Serial.printf("Duration: %d\n", millis() - start_ms);
    }
  } else {
    delay(100);
  }
}
