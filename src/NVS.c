#include "nvs_flash.h"
#include "nvs.h"

void guardar_calibracion(float offset_pitch, float offset_roll) {
    nvs_handle_t handle;
    nvs_open("calib", NVS_READWRITE, &handle);
    nvs_set_blob(handle, "pitch_off", &offset_pitch, sizeof(float));
    nvs_set_blob(handle, "roll_off", &offset_roll, sizeof(float));
    nvs_commit(handle);
    nvs_close(handle);
}

void leer_calibracion(float *offset_pitch, float *offset_roll) {
    nvs_handle_t handle;
    size_t len = sizeof(float);
    nvs_open("calib", NVS_READONLY, &handle);
    nvs_get_blob(handle, "pitch_off", offset_pitch, &len);
    nvs_get_blob(handle, "roll_off", offset_roll, &len);
    nvs_close(handle);
}