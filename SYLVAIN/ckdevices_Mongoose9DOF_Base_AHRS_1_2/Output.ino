/*
Trame générée :
R  Roll  P Pitch  Y Yaw A altitude S


void printdata(void)
{ 
  long int h;
  float pression;
 

      pression=sen_data.baro_pres;
      h=((pression-101325)/-12)*100;
      
      
      Serial.print("R");
      Serial.print((int)ToDeg(roll));
      Serial.print("P");
      Serial.print((int)ToDeg(pitch));
      Serial.print("Y");
      Serial.print((int)ToDeg(yaw));
      
      Serial.print("A");
      Serial.print(h);
      Serial.print("S");
     
      return;
}

