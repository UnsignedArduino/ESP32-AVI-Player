#include <Arduino.h>
#include <M5Core2.h>
#include <WiFi.h>
#include <avilib.h>
#include <cinepak.h>

static avi_t* a;
static long frames, estimateBufferSize, aRate, aBytes, aChunks, actual_video_size;
static int w, h, aChans, aBits, aFormat;
static double fr;
static char* compressor;
static char* vidbuf;
static char* audbuf;
static uint16_t* output_buf;
static size_t output_buf_size;
static bool isStopped = true;
static int curr_frame = 0;
static long curr_chunk = 0;
static int skipped_frames = 0;
static bool skipped_last_frame = false;
static unsigned long start_ms, next_frame_ms;

const char* avi_file = "/video.avi";

CinepakDecoder decoder;

void setup() {
  M5.begin();
  WiFi.mode(WIFI_OFF);

  Serial.println("avilib");

  a = AVI_open_input_file(avi_file, 1);

  if (a) {
    frames = AVI_video_frames(a);
    w = AVI_video_width(a);
    h = AVI_video_height(a);
    fr = AVI_frame_rate(a);
    compressor = AVI_video_compressor(a);
    estimateBufferSize = w * h * 4 / 5;
    Serial.printf("AVI frames: %d, %d x %d @ %.2f fps, format: %s, estimateBufferSize: %d, ESP.getFreeHeap(): %d\n",
                  frames, w, h, fr, compressor, estimateBufferSize, ESP.getFreeHeap());

    aChans = AVI_audio_channels(a);
    aBits = AVI_audio_bits(a);
    aFormat = AVI_audio_format(a);
    aRate = AVI_audio_rate(a);
    aBytes = AVI_audio_bytes(a);
    aChunks = AVI_audio_chunks(a);
    Serial.printf("Audio channels: %d, bits: %d, format: %d, rate: %d, bytes: %d, chunks: %d\n", aChans, aBits, aFormat,
                  aRate, aBytes, aChunks);

    vidbuf = (char*)heap_caps_malloc(estimateBufferSize, MALLOC_CAP_8BIT);
    audbuf = (char*)heap_caps_malloc(1024, MALLOC_CAP_8BIT);
    output_buf_size = w * h * 2;
    output_buf = (uint16_t*)malloc(output_buf_size);

    isStopped = false;
    start_ms = millis();
    next_frame_ms = start_ms + ((curr_frame + 1) * 1000 / fr);
  }
}

void loop() {
  if (!isStopped) {
    if (curr_frame < frames) {
      long audio_bytes = AVI_audio_size(a, curr_chunk++);
      AVI_read_audio(a, audbuf, audio_bytes);

      if (millis() < next_frame_ms) // check show frame or skip frame
      {
        AVI_set_video_position(a, curr_frame);

        int iskeyframe;
        long video_bytes = AVI_frame_size(a, curr_frame);
        if (video_bytes > estimateBufferSize) {
          Serial.printf("video_bytes(%d) > estimateBufferSize(%d)\n", video_bytes, estimateBufferSize);
        } else {
          actual_video_size = AVI_read_frame(a, vidbuf, &iskeyframe);
          // Serial.printf("frame: %d, iskeyframe: %d, video_bytes: %d, actual_video_size: %d, audio_bytes: %d,
          // ESP.getFreeHeap(): %d\n", curr_frame, iskeyframe, video_bytes, actual_video_size, audio_bytes,
          // ESP.getFreeHeap());

          if ((!skipped_last_frame) || iskeyframe) {
            skipped_last_frame = false;
            decoder.decodeFrame((uint8_t*)vidbuf, actual_video_size, output_buf, output_buf_size);
            // gfx->draw16bitBeRGBBitmap(0, 0, output_buf, w, h);
            // M5.Lcd.drawBitmap(0, 0, w, h, output_buf);
            bool swap = M5.Lcd.getSwapBytes();
            M5.Lcd.setSwapBytes(false);
            M5.Lcd.pushImage(0, 0, w, h, output_buf);
            M5.Lcd.setSwapBytes(swap);
          }
          else
          {
            ++skipped_frames;
          }
        }
        while (millis() < next_frame_ms) {
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      } else {
        ++skipped_frames;
        skipped_last_frame = true;
        Serial.printf("Skip frame %d > %d\n", millis(), next_frame_ms);
      }

      ++curr_frame;
      next_frame_ms = start_ms + ((curr_frame + 1) * 1000 / fr);
    } else {
      AVI_close(a);
      isStopped = true;
      Serial.printf("Duration: %d, skipped frames: %d\n", millis() - start_ms, skipped_frames);
    }
  } else {
    delay(100);
  }
}
