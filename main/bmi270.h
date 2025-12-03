#ifndef BMI270_H
#define BMI270_H

#include "esp_err.h"

typedef struct {
    float ax, ay, az; // Acelerómetro
    float gx, gy, gz; // Giroscopio
} bmi270_data_t;

esp_err_t bmi270_init(void);

/**
 * @brief Lee los datos del acelerómetro y giroscopio si están listos.
 *
 * Comprueba el bit de 'data ready'. Si está activo, lee los 12 bytes
 * de datos, los convierte a float (m/s^2 y rad/s) y los
 * almacena en la estructura de datos.
 *
 * @param data Puntero a la estructura donde se guardarán los datos.
 * @return esp_err_t
 * - ESP_OK: Si los datos se leyeron correctamente.
 * - ESP_ERR_NOT_FOUND: Si no había datos listos para leer.
 * - Otro error: Si falló la comunicación I2C.
 */
esp_err_t bmi270_read_data(bmi270_data_t *data);

#endif // BMI270_Hs