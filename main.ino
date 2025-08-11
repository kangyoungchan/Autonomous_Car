#define moterEna1 5
#define moterEnb1 6
#define moterEna2 9
#define moterEnb2 10


void setup() {
  pinMode(moterEna1, OUTPUT);
  pinMode(moterEnb1, OUTPUT);
  pinMode(moterEna2, OUTPUT);
  pinMode(moterEnb2, OUTPUT);
}

void loop() {
  
analogWrite(moterEna1, 95);
analogWrite(moterEnb1, 95);
analogWrite(moterEna2, 95);
analogWrite(moterEnb2, 95);

}
