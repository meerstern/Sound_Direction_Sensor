#include <Wire.h>

#define SDS_SLAVE_ADDRESS	0x30
#define SDS_ACTVAL_REG		0x04



void setup()
{
  float ActivityValueThreshold=1.0f;
  int 	actset;
  byte writeValArry[4];
  
  actset=(int)(ActivityValueThreshold*10);
  
  writeValArry[0]=SDS_ACTVAL_REG;
  writeValArry[1]=(actset>>8)&0xFF;
  writeValArry[2]=(actset>>0)&0xFF;
  
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);
  
  //Override Activity Threshold Value ( VR is ignored )
  Wire.beginTransmission(SDS_SLAVE_ADDRESS); 
  Wire.write(writeValArry,3);  
  Wire.endTransmission();
	  
  
  Serial.println("\nStart Sounc Direction Sensor");
}


void loop()
{
	int index;
	byte valArry[8];
	float degree;
	float degreeOrg;
	float activity;
	bool  newValue;
	
	Wire.beginTransmission(SDS_SLAVE_ADDRESS);
	Wire.write(0);
	Wire.endTransmission(false);
	Wire.requestFrom(SDS_SLAVE_ADDRESS, 8, true);

	for(index = 0; index < 8; index++ ) 
		valArry[index]=Wire.read();

	degree		= ((int)valArry[0]*256+(int)valArry[1])/10.0f;
	degreeOrg	= ((int)valArry[2]*256+(int)valArry[3])/10.0f;
	activity 	= ((int)valArry[4]*256+(int)valArry[5])/10.0f;
	newValue 	= valArry[6];
	
	if(newValue==true)
	{
		Serial.print("deg:");
		Serial.print(degree);
		Serial.print("\t act:");
		Serial.print(activity);
		Serial.println("");
	}
	delay(50);  
}
