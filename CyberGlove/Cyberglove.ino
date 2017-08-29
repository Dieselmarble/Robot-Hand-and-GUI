int pin_1 = A0;     // potentiometer wiper
int pin_2 = A1;
int pin_3 = A2;
int val_1 = 0;
int val_2 = 0;
int val_3 = 0;
char userInput;
void setup()

{

  Serial.begin(9600);          //  setup serial

}


void loop(){
  if (Serial.available() > 0){
      userInput = Serial.read();
	 if (userInput == 'y'){	
            val_1 = analogRead(pin_1);    // read the input pin
            val_2 = analogRead(pin_2);
            val_3 = analogRead(pin_3); 
            int val[]={val_1,val_2,val_3};
            for (int i=0;i<=2;i++){
    	       Serial.println(val[i]); 
	       }
           } 
     }
}
