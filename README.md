# ConstSpeedStepGen
Sabit hızlı gerçek zamanlı step/dir/enable(opsiyonel) sinyali ile çalışan step motor kontrolü arduino projesidir.


#DualMotorShield.pde örnek kod;

// ConstSpeedStepGen
// step ve dir ile çalışan (opsiyonel enable pini destekler) 
// sabit hızlı (ivme rampasız) realtime step motor kütüphanesidir kütüphnesi örneğidir.
// Yazar Eyüp Fındıklı eyupfindikli@gmail.com  2016 12 25 

// 
#include <ConstSpeedStepGen.h>

// The X Stepper pins
#define STEPPER1_DIR_PIN 3
#define STEPPER1_STEP_PIN 2
// The Y stepper pins
#define STEPPER2_DIR_PIN 7
#define STEPPER2_STEP_PIN 6

// Define some steppers and the pins the will use
ConstSpeedStepGen stepper1(STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
ConstSpeedStepGen stepper2(STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

void setup()
{  
    stepper1.setMaxSpeed(200.0);
    stepper1.setAcceleration(200.0);
    stepper1.moveTo(100);
    
    stepper2.setMaxSpeed(100.0);
    stepper2.setAcceleration(100.0);
    stepper2.moveTo(100);
}

void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
	stepper1.moveTo(-stepper1.currentPosition());
    if (stepper2.distanceToGo() == 0)
	stepper2.moveTo(-stepper2.currentPosition());
    stepper1.run(); // motorları hareket edebilmesi için sürekli olabildiğince çalıştırılmalıdır
    stepper2.run(); // bu durumda minimu step genişliği step puls width göz önünde bulundurulmalıdır min 200ns
}
