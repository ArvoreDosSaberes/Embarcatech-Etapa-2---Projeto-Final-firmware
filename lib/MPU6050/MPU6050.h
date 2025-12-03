#ifndef MPU6050__H
#define MPU6050__H

/**
 * @file MPU6050/MPU6050.h
 * @brief Driver de alto nível para o sensor inercial MPU6050 usando o proxy I2C.
 *
 * Este arquivo define a classe `MPU6050`, responsável por configurar o sensor,
 * efetuar leituras de aceleração, giroscópio e temperatura, comunicando-se via
 * a classe proxy `I2C` (ver `I2C/I2C.h`).
 *
 * Fluxo típico de uso:
 * - Criar uma instância de `I2C` parametrizada com a instância do periférico e pinos SDA/SCL
 * - Chamar `I2C::begin()` e opcionalmente `I2C::setClock()`
 * - Instanciar `MPU6050` passando o ponteiro para `I2C`
 * - Chamar `MPU6050::begin()`
 * - Ler dados com `getAccelleration()`, `getGyroscope()` e `getTemp()`
 */

#include <cstdint>

// Forward declaration para evitar dependência direta de include path no header
class I2C;

/**
 * @class MPU6050
 * @brief Classe que encapsula o acesso ao sensor MPU6050 via I2C.
 */
class MPU6050 {

  public:

    /**
     * @brief Estrutura para representar um vetor 3D (x, y, z).
     */
    typedef struct {
      float x;
      float y;
      float z;
    } VECT_3D;

    /**
     * @brief Endereço I2C padrão do MPU6050 quando AD0=0.
     */
    static const uint8_t I2C_ADDR = 0x68;

    /**
     * @brief Valor esperado do registrador WHO_AM_I.
     */
    static const uint8_t ID = 0x68;

    /**
     * @brief Construtor do driver.
     * @param i2c Ponteiro para a instância do proxy I2C.
     * @param addr Endereço I2C do dispositivo (padrão: 0x68).
     */
    MPU6050 (I2C *i2c, uint8_t addr = MPU6050::I2C_ADDR);

    /**
     * @brief Inicializa o sensor e aplica a configuração padrão.
     */
    void    begin(void);

    /**
     * @brief Obtém o ID (WHO_AM_I) lido do sensor.
     * @return Valor de identificação do dispositivo.
     */
    uint8_t getId(void);

    /**
     * @brief Executa um reset do dispositivo e dos caminhos de sinal.
     */
    void    reset(void);

    /**
     * @brief Lê e retorna a aceleração em g.
     * @param vect Ponteiro de saída para receber x, y, z.
     * @param scale Fator de escala (padrão 16384.0 para ±2g).
     */
    void    getAccelleration(VECT_3D *vect, float scale = 16384.0);

    /**
     * @brief Lê e retorna a velocidade angular em °/s.
     * @param vect Ponteiro de saída para receber x, y, z.
     * @param scale Fator de escala (padrão 65.5 para ±500°/s).
     */
    void    getGyroscope(VECT_3D *vect, float scale = 65.5);

    /**
     * @brief Lê e retorna a temperatura interna em °C.
     * @return Temperatura em graus Celsius.
     */
    float   getTemp();

  private:
    
    // Registradores internos do MPU6050
    static const int SMPLRT_DIV = 0x19;
    static const int CONFIG = 0x1A;
    static const int GYRO_CONFIG = 0x1B;
    static const int ACCEL_CONFIG = 0x1C;
    static const int ACCEL_OUT = 0x3B;
    static const int TEMP_OUT = 0x41;
    static const int GYRO_OUT = 0x43;
    static const int SIG_PATH_RESET = 0x68;
    static const int PWR_MGMT_1 = 0x6B;
    static const int PWR_MGMT_2 = 0x6C;
    static const int WHO_AM_I = 0x75;

    /** Ponteiro para o proxy I2C. */
    I2C *i2c;
    /** Endereço I2C atual do dispositivo. */
    uint8_t addr;
    /** ID lido do registrador WHO_AM_I. */
    uint8_t id;

    /** Buffer de dados brutos lidos do dispositivo. */
    int16_t raw[14];

    /**
     * @brief Efetua leitura dos 14 bytes de dados a partir de `ACCEL_OUT`.
     */
    void     readRaw(void);

    /**
     * @brief Lê um byte de um registrador.
     * @param reg Endereço do registrador.
     * @return Valor lido.
     */
    uint8_t  read8(uint8_t reg);

    /**
     * @brief Escreve um byte em um registrador.
     * @param reg Endereço do registrador.
     * @param val Valor a ser escrito.
     */
    void     write8(uint8_t reg, uint8_t val);
};


#endif // MPU6050__H