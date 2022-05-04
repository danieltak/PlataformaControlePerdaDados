
float x;
float x2;
int datap = 0;
float um = 0;
float um2 = 0;
float zero = 0;
float zero2 = 0;
int inByte = 0;
int counter;
float p;
float p2;
int erro;
float k;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  k = 0.7;
}

void loop() {


  if (Serial.available() > 0)
  {
    // then read the first available byte
    inByte = Serial.read();

    switch (inByte)
    {
      case'a':
        datap = 1;
        break;
      case 's':
        datap = 0;
        um = 0;
        um2 = 0;
        zero = 0;
        zero2 = 0;
        counter = 0;
        x = 0;
        erro;
        break;
    }
  }


  if (datap == 1) {
    x = bernoulli(k);
    x2 = bernoulli2(k);
    counter = counter + 1;

    if (x == 1) {
      um = um + 1;
    } else if (x == 0) {
      zero = zero + 1;
    } else {
      erro = erro + 1;
    }

    if (x2 == 1) {
      um2 = um2 + 1;
    } else if (x2 == 0) {
      zero2 = zero2 + 1;
    } else {
      erro = erro + 1;
    }

    if (x2==-1 || x==-1){
      erro=erro+1;
    }
    p = zero / (um+zero);
    p2 = zero2 / (um2+zero2);

    
    Serial.print(counter);
    Serial.print("\t");
    Serial.print(p, 4);
    Serial.print("\t");
    Serial.print(p2, 4);
    Serial.print("\t");
    Serial.print(erro);
    Serial.print("\t");
    Serial.print(um);
    Serial.print("\t");
    Serial.print(zero);
    Serial.print("\t");
    Serial.print(um2);
    Serial.print("\t");
    Serial.println(zero2);
    delay(5);
  }

  if (counter == 1200) {
    datap = 0;
  }
}

int bernoulli(float p) {
  if (p < 0 || p > 1) return -1;
  float x = (float)rand() / (float)(RAND_MAX / 1);
  if (p < x) return 1;
  return 0;
}

int bernoulli2(float p) {
  if (p < 0 || p > 1) return -1;
  float x = random(RAND_MAX) / float(RAND_MAX / 1);
  if (p < x) return 1;
  return 0;
}
