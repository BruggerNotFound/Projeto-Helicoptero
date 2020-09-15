Código YPR
=====================

O código YPR (Yaw, Pitch e Roll) utiliza a bússola e o acelerômetro para calcular a orientação e inclinação do dispositivo no formato Yaw, Pitch e Roll;
além de informar a direção que a bússola aponta.

Bibliotecas:  
--------
  - 12Cdev (https://github.com/jrowberg/i2cdevlib)
  - MPU6050 (de I2Cdev)
  - QMC5883LC (https://github.com/mprograms/QMC5883LCompass)
  
Detalhes:
--------
  Para obter os valores de Yaw, Pitch e Roll, o código utiliza o acelerômetro/giroscópio e uma bússola.
  Vale notar que o acelerômetro sofre de um efeito de "drift" e perde certa precisão com o passar do tempo, o valor de Yaw não sofre este efeito pois a bússola serve como uma solução a longo prazo para o problema do drift, mas os valores de Pitch e Roll são afetados. 
  Para baratear o projeto a bússola poderia ser removida, então seria usado o Digital Motion Processor, já incluso no MPU6050 que é usado para fazer cálculos na própria placa e é capaz de eliminar o drift que ocorre no Yaw semelhante a como uma bússola faria, mas o código não inclui essa lógica.
