/*
 * Test to flip a single dot both ways
 * 10/1/12
 */

const int columnPositiveChannel = 5;
const int columnNegativeChannel = 4;
const int rowNegativeChannel = 3;
const int rowPositiveChannel = 2;


void setup()
{
  Serial.begin(9600);
  pinMode(columnPositiveChannel, OUTPUT);
  pinMode(columnNegativeChannel, OUTPUT);
  pinMode(rowNegativeChannel, OUTPUT);
  pinMode(rowPositiveChannel, OUTPUT);
  
  Serial.println("Beginning...");
}

void loop()
{

channelsOff();
  
  delay(500);
  
  Serial.println("Column + Row -");
  digitalWrite(columnPositiveChannel, HIGH);
  digitalWrite(columnNegativeChannel, LOW);
  digitalWrite(rowNegativeChannel, HIGH);
  digitalWrite(rowPositiveChannel, LOW);
  
  delay(800);
  channelsOff();
  
  delay(500);
  
  Serial.println("Column - Row +");
  digitalWrite(columnPositiveChannel, LOW);
  digitalWrite(columnNegativeChannel, HIGH);
  digitalWrite(rowNegativeChannel, LOW);
  digitalWrite(rowPositiveChannel, HIGH);
  
  delay(800);
  channelsOff();
  
    }
    
    
//function to turn all channels off
void channelsOff()
{
    Serial.println("All channels off");
  digitalWrite(columnPositiveChannel, LOW);
  digitalWrite(columnNegativeChannel, LOW);
  digitalWrite(rowNegativeChannel, LOW);
  digitalWrite(rowPositiveChannel, LOW);
  
}

  

      
    
