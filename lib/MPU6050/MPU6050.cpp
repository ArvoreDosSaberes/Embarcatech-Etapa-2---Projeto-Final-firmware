/**
 * @file MPU6050/MPU6050.cpp
 * @brief Implementação do driver do sensor MPU6050 utilizando o proxy I2C.
 *
 * Este arquivo contém a implementação dos métodos declarados em
 * `MPU6050/MPU6050.h`. A comunicação é realizada por meio da classe `I2C`.
 */
#include "MPU6050.h"
#include "I2C.hpp"
#include "log_vt100.h"
#include "pico/stdlib.h" // sleep_ms
#include <cstdint>
#include <hardware/timer.h>

#if defined(I2C_USE_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif

/**
 * @brief Construtor: armazena ponteiro para o proxy I2C e endereço.
 * @param i2c Ponteiro para a interface I2C a ser utilizada.
 * @param addr Endereço I2C do dispositivo.
 */
MPU6050::MPU6050 (I2C *i2c, uint8_t addr) {

  this->i2c = i2c;
  this->addr = addr;
}

/**
 * @brief Inicializa o sensor com configuração padrão.
 */
void MPU6050::begin() {
  LOG_INFO("[MPU6050.begin] Inicializando...");

  LOG_INFO("[MPU6050.begin] Executando reset...");
  reset();
  this->id = read8(WHO_AM_I);
  LOG_INFO("[MPU6050.begin] ID lido: 0x%02X", this->id);
  LOG_INFO("[MPU6050.begin] Aplicando configurações...");
  write8(CONFIG, 0x00);
  busy_wait_ms(100);
  write8(ACCEL_CONFIG, 0x00);
  write8(GYRO_CONFIG, 0x08);
  write8(SMPLRT_DIV, 0x00);
  write8(PWR_MGMT_1, 0x01);
  write8(PWR_MGMT_2, 0x00);
  busy_wait_ms(20);
  LOG_INFO("[MPU6050.begin] Configurações aplicadas");

}

/**
 * @brief Retorna o ID (WHO_AM_I) lido do sensor.
 */
uint8_t MPU6050::getId() {
  return id;
}

/**
 * @brief Executa reset do dispositivo e dos caminhos de sinal.
 */
void MPU6050::reset(void) {
  uint8_t val = read8(PWR_MGMT_1) | 0x80;
  write8(PWR_MGMT_1, val);
  while (val & 0x80) {
    val = read8(PWR_MGMT_1);
    busy_wait_ms(1);
  }
  val = read8(SIG_PATH_RESET) | 0x07;
  write8(SIG_PATH_RESET, val);
  while (val & 0x07) {
    val = read8(SIG_PATH_RESET);
    busy_wait_ms(1);
  }

  LOG_INFO("[MPU6050.reset] Reset completo");

}

/**
 * @brief Lê aceleração em g e preenche o vetor informado.
 * @param vect Estrutura de saída x/y/z.
 * @param scale Fator de escala aplicado aos dados brutos.
 */
void MPU6050::getAccelleration(VECT_3D *vect, float scale) {
  readRaw();
  vect->x = (float) raw[0] / scale;
  vect->y = (float) raw[1] / scale;
  vect->z = (float) raw[2] / scale;
}

/**
 * @brief Lê giroscópio em °/s e preenche o vetor informado.
 * @param vect Estrutura de saída x/y/z.
 * @param scale Fator de escala aplicado aos dados brutos.
 */
void MPU6050::getGyroscope(VECT_3D *vect, float scale) {
  // NOTA: Não usar taskENTER_CRITICAL aqui pois readRaw usa I2C com semáforo
  // A proteção do barramento I2C é feita pelo semáforo em I2C::beginTransmission
  readRaw();
  vect->x = (float) raw[4] / scale;
  vect->y = (float) raw[5] / scale;
  vect->z = (float) raw[6] / scale;
}

/**
 * @brief Lê temperatura interna e converte para °C.
 * @return Temperatura em graus Celsius.
 */
float MPU6050::getTemp() {
  // NOTA: Não usar taskENTER_CRITICAL aqui pois readRaw usa I2C com semáforo
  // A proteção do barramento I2C é feita pelo semáforo em I2C::beginTransmission
  readRaw();
  float temp = (float) raw[3]/340.0 + 36.53;
  return temp;
}

/**
 * @brief Efetua leitura contínua de 14 bytes a partir de ACCEL_OUT.
 */
void MPU6050::readRaw() {

  LOG_INFO("[MPU6050.readRaw] Lendo dados brutos...");
  uint8_t data[14];

  uint8_t reg = ACCEL_OUT;
  i2c->beginTransmission(addr);
  i2c->write(reg);

  i2c->requestFrom(addr, 14);
  for (int i = 0; i < 14; ++i) {
    int v = i2c->read();
    data[i] = (v < 0) ? 0 : (uint8_t)v;
  }
  i2c->endTransmission();

  for (int i = 0; i < 7; i++) {
    raw[i] = (data[2*i] << 8) | data[2*i+1];
  }

  LOG_INFO("[MPU6050.readRaw] Dados brutos lidos: %d %d %d %d %d %d %d", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]);

}

/**
 * @brief Lê um byte de um registrador.
 * @param reg Endereço do registrador.
 * @return Valor lido.
 */
uint8_t MPU6050::read8 (uint8_t reg)  {  
  uint8_t val[1];  
    
  i2c->beginTransmission(addr);
  i2c->write(reg);

  i2c->requestFrom(addr, 1);
  int v = i2c->read();
  val[0] = (v < 0) ? 0 : (uint8_t)v;
  i2c->endTransmission();
  return val[0];
} 

/**
 * @brief Escreve um byte em um registrador.
 * @param reg Endereço do registrador.
 * @param val Valor a ser escrito.
 */
void MPU6050::write8 (uint8_t reg, uint8_t val)  {  
  
  uint8_t aux[2];

  aux[0] = reg;
  aux[1] = val;
  i2c->beginTransmission(addr);
  i2c->write(aux, 2);
  i2c->endTransmission();
  
} 
