#include "nvs_flash.h"
#include "nvs.h"

void guardar_calibracion(int offsetX, int offsetY, int offsetZ, int gainX, int gainY, int gainZ) {
    nvs_handle_t handle;
    nvs_open("calib", NVS_READWRITE, &handle);
    nvs_set_blob(handle, "offsetX", &offsetX, sizeof(int));
    nvs_set_blob(handle, "offsetY", &offsetY, sizeof(int));
    nvs_set_blob(handle, "offsetZ", &offsetZ, sizeof(int));
    nvs_set_blob(handle, "gainX", &gainX, sizeof(int));
    nvs_set_blob(handle, "gainY", &gainY, sizeof(int));
    nvs_set_blob(handle, "gainZ", &gainZ, sizeof(int));
    nvs_commit(handle);
    nvs_close(handle);
}

// Devuelve ESP_OK solo si encontro una calibracion previa guardada en NVS.
// ESP_ERR_NVS_NOT_FOUND (namespace o clave inexistente) indica que el
// equipo todavia no fue calibrado nunca, p.ej. primer arranque de fabrica.
esp_err_t leer_calibracion(int *offsetX, int *offsetY, int *offsetZ, int *gainX, int *gainY, int *gainZ) {
    nvs_handle_t handle;
    size_t len = sizeof(int);
    esp_err_t err = nvs_open("calib", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }
    if (err == ESP_OK) err = nvs_get_blob(handle, "offsetX", offsetX, &len);
    if (err == ESP_OK) err = nvs_get_blob(handle, "offsetY", offsetY, &len);
    if (err == ESP_OK) err = nvs_get_blob(handle, "offsetZ", offsetZ, &len);
    if (err == ESP_OK) err = nvs_get_blob(handle, "gainX", gainX, &len);
    if (err == ESP_OK) err = nvs_get_blob(handle, "gainY", gainY, &len);
    if (err == ESP_OK) err = nvs_get_blob(handle, "gainZ", gainZ, &len);
    nvs_close(handle);
    return err;
}