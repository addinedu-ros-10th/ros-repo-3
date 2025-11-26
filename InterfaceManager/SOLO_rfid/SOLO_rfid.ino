#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 9
#define SS_PIN 10

MFRC522 rfid(SS_PIN, RST_PIN);

byte USER1_UID[] = {0x29, 0xBC, 0x3E, 0x02};
byte USER2_UID[] = {0x94, 0x1F, 0x07, 0x05};
byte UID_SIZE = 4;

bool compareUID(byte *uid1, byte *uid2) {
  for (byte i = 0; i < UID_SIZE; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}

void printHexString(byte *uid, byte size) {
  for (byte i = 0; i < size; i++) {
    if (uid[i] < 0x10) Serial.print("0");
    Serial.print(uid[i], HEX);
    if (i < size - 1) Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  Serial.println("RFID Ready. Tap card...");
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
    return;

  byte *uid = rfid.uid.uidByte;

  Serial.print("HEX UID: ");
  printHexString(uid, rfid.uid.size);

  byte converted_uid[4] = {0x00, 0x00, 0x00, 0x00};

  if (compareUID(uid, USER1_UID)) {
    converted_uid[3] = 0x01;    // 00 00 00 01
    Serial.println("사용자 1 인증됨 → 0x00000001 로 변환");
  }
  else if (compareUID(uid, USER2_UID)) {
    converted_uid[3] = 0x02;    // 00 00 00 02
    Serial.println("사용자 2 인증됨 → 0x00000002 로 변환");
  }
  else {
    Serial.println("미등록 UID → 변환 불가");
    return;
  }

  Serial.print("Converted UID (HEX): ");
  printHexString(converted_uid, 4);

  Serial.print("Converted RAW UID sent: ");
  printHexString(converted_uid, 4);

  Serial.write(converted_uid, 4);  // 4바이트 그대로 전송
  Serial.println();

  delay(1000);
}
