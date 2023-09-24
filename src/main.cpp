#include <Arduino.h>
#include <M5Core2.h>
#include <MP3DecoderHelix.h>
#include <WiFi.h>
#include <avilib.h>
#include <cinepak.h>
#include <driver/i2s.h>

avi_t* a;
long frames, estimateBufferSize, aRate, aBytes, aChunks, actual_video_size;
int w, h, aChans, aBits, aFormat;
double fr;
char* compressor;
char* vidbuf;
char* audbuf;
size_t audbuf_read;
size_t audbuf_remain = 0;
uint16_t* output_buf;
size_t output_buf_size;
bool isStopped = true;
int curr_frame = 0;
int skipped_frames = 0;
bool skipped_last_frame = false;
unsigned long start_ms, curr_ms, next_frame_ms;
unsigned long total_read_video_ms = 0;
unsigned long total_decode_video_ms = 0;
unsigned long total_show_video_ms = 0;
unsigned long total_read_audio_ms = 0;
unsigned long total_decode_audio_ms = 0;
unsigned long total_play_audio_ms = 0;

#define GAIN_LEVEL 0.8

const char* avi_file = "/video.avi";

CinepakDecoder decoder;

int _samprate = 0;
void audioDataCallback(MP3FrameInfo& info, int16_t* pwm_buffer, size_t len, void* ref) {
  unsigned long s = millis();
  if (_samprate != info.samprate) {
    Serial.printf("bitrate: %d, nChans: %d, samprate: %d, bitsPerSample: %d, outputSamps: %d, layer: %d, version: %d",
                  info.bitrate, info.nChans, info.samprate, info.bitsPerSample, info.outputSamps, info.layer,
                  info.version);
    i2s_set_clk(Speak_I2S_NUMBER, info.samprate /* sample_rate */, info.bitsPerSample /* bits_cfg */,
                (info.nChans == 2) ? I2S_CHANNEL_STEREO : I2S_CHANNEL_MONO /* channel */);
    _samprate = info.samprate;
  }
  for (int i = 0; i < len; i++) {
    pwm_buffer[i] = pwm_buffer[i] * GAIN_LEVEL;
  }
  size_t i2s_bytes_written = 0;
  i2s_write(Speak_I2S_NUMBER, pwm_buffer, len * 2, &i2s_bytes_written, portMAX_DELAY);
  // log_i("len: %d, i2s_bytes_written: %d", len, i2s_bytes_written);
  total_play_audio_ms += millis() - s;
}

libhelix::MP3DecoderHelix mp3(audioDataCallback);

void mp3_player_task(void* pvParam) {
  unsigned long ms = millis();
  char* p;
  long w;
  do {
    ms = millis();

    p = audbuf;
    while (audbuf_remain > 0) {
      w = mp3.write(p, audbuf_remain);
      // log_i("r: %d, w: %d\n", r, w);
      audbuf_remain -= w;
      p += w;
    }
    total_decode_audio_ms += millis() - ms;
    ms = millis();
    vTaskDelay(pdMS_TO_TICKS(1));
  } while (audbuf_read > 0);

  log_i("MP3 stop.");

  i2s_zero_dma_buffer(I2S_NUM_0);
  vTaskDelete(NULL);
}

BaseType_t mp3_player_task_start(avi_t* a) {
  mp3.begin();

  return xTaskCreatePinnedToCore((TaskFunction_t)mp3_player_task, (const char* const)"MP3 Player Task",
                                 (const uint32_t)2000, (void* const)a, (UBaseType_t)configMAX_PRIORITIES - 1,
                                 (TaskHandle_t* const)NULL, (const BaseType_t)0);
}

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
    estimateBufferSize = w * h * 2 / 5;
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
    audbuf = (char*)heap_caps_malloc(MP3_MAX_FRAME_SIZE, MALLOC_CAP_8BIT);
    output_buf_size = w * h * 2;
    output_buf = (uint16_t*)malloc(output_buf_size);

    isStopped = false;
    start_ms = millis();
    next_frame_ms = start_ms + ((curr_frame + 1) * 1000 / fr);

    Serial.println("Play AVI start");
    curr_ms = millis();
    start_ms = curr_ms;

    audbuf_read = AVI_read_audio(a, audbuf, MP3_MAX_FRAME_SIZE);
    audbuf_remain = audbuf_read;
    total_read_audio_ms += millis() - curr_ms;
    curr_ms = millis();

    Serial.println("Start play audio task");
    BaseType_t ret_val = mp3_player_task_start(a);
    if (ret_val != pdPASS) {
      Serial.printf("mp3_player_task_start failed: %d\n", ret_val);
    }
  }
}

void loop() {
  M5.update();
  if (!isStopped) {
    if (curr_frame < frames) {
      if (audbuf_remain == 0) {
        audbuf_read = AVI_read_audio(a, audbuf, MP3_MAX_FRAME_SIZE);
        audbuf_remain = audbuf_read;
        total_read_audio_ms += millis() - curr_ms;
        curr_ms = millis();
      }

      if (millis() < next_frame_ms) // check show frame or skip frame
      {
        AVI_set_video_position(a, curr_frame);

        int iskeyframe;
        long video_bytes = AVI_frame_size(a, curr_frame);
        if (video_bytes > estimateBufferSize) {
          Serial.printf("video_bytes(%d) > estimateBufferSize(%d)\n", video_bytes, estimateBufferSize);
        } else {
          actual_video_size = AVI_read_frame(a, vidbuf, &iskeyframe);
          total_read_video_ms += millis() - curr_ms;
          curr_ms = millis();
          // Serial.printf("frame: %d, iskeyframe: %d, video_bytes: %d, actual_video_size: %d, audio_bytes: %d,
          // ESP.getFreeHeap(): %d\n", curr_frame, iskeyframe, video_bytes, actual_video_size, audio_bytes,
          // ESP.getFreeHeap());

          // if ((!skipped_last_frame) || iskeyframe)
          {
            // skipped_last_frame = false;
            decoder.decodeFrame((uint8_t*)vidbuf, actual_video_size, output_buf, output_buf_size);
            // gfx->draw16bitBeRGBBitmap(0, 0, output_buf, w, h);
            // M5.Lcd.drawBitmap(0, 0, w, h, output_buf);
            bool swap = M5.Lcd.getSwapBytes();
            M5.Lcd.setSwapBytes(false);
            M5.Lcd.pushImage(0, 0, w, h, output_buf);
            M5.Lcd.setSwapBytes(swap);
          }
          // else
          // {
          //   ++skipped_frames;
          // }
        }
        while (millis() < next_frame_ms) {
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      }
      else
      {
        ++skipped_frames;
        skipped_last_frame = true;
        // Serial.printf("Skip frame %d > %d\n", millis(), next_frame_ms);
      }

      ++curr_frame;
      curr_ms = millis();
      next_frame_ms = start_ms + ((curr_frame + 1) * 1000 / fr);
    } else {
      int time_used = millis() - start_ms;
      int total_frames = curr_frame;
      audbuf_read = 0;
      AVI_close(a);
      isStopped = true;
      Serial.println("Play AVI end");

      int played_frames = total_frames - skipped_frames;
      float fps = 1000.0 * played_frames / time_used;
      total_decode_audio_ms -= total_play_audio_ms;
      total_decode_video_ms -= total_show_video_ms;
      Serial.printf("Played frames: %d\n", played_frames);
      Serial.printf("Skipped frames: %d (%0.1f %%)\n", skipped_frames, 100.0 * skipped_frames / total_frames);
      Serial.printf("Time used: %d ms\n", time_used);
      Serial.printf("Expected FPS: %0.1f\n", fr);
      Serial.printf("Actual FPS: %0.1f\n", fps);
      Serial.printf("Read audio: %lu ms (%0.1f %%)\n", total_read_audio_ms, 100.0 * total_read_audio_ms / time_used);
      Serial.printf("Decode audio: %lu ms (%0.1f %%)\n", total_decode_audio_ms,
                    100.0 * total_decode_audio_ms / time_used);
      Serial.printf("Play audio: %lu ms (%0.1f %%)\n", total_play_audio_ms, 100.0 * total_play_audio_ms / time_used);
      Serial.printf("Read video: %lu ms (%0.1f %%)\n", total_read_video_ms, 100.0 * total_read_video_ms / time_used);
      Serial.printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video_ms,
                    100.0 * total_decode_video_ms / time_used);
      Serial.printf("Show video: %lu ms (%0.1f %%)\n", total_show_video_ms, 100.0 * total_show_video_ms / time_used);
    }
  } else {
    delay(100);
  }
}
