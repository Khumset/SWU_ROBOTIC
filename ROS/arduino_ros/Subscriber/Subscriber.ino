#include <ros.h> // นำเข้า header file สำหรับใช้งาน ROS บน Arduino
#include <std_msgs/String.h> // นำเข้า header file สำหรับใช้งานข้อความประเภท String จาก ROS

ros::NodeHandle nh; // สร้างตัวแปร nh สำหรับจัดการ node ใน ROS

void chatterCallback(const std_msgs::String& msg) {
  // ฟังก์ชันนี้จะถูกเรียกเมื่อมีข้อความใหม่ถูกส่งมาจาก topic ชื่อ chatter
  String listen = msg.data; // เก็บข้อความที่ได้รับในตัวแปร listen
  Serial.println(listen); // พิมพ์ข้อความที่ได้รับออกทาง serial monitor

  // หากมีข้อความถูกส่งเข้ามาให้หลอดไฟที่ขา 13 กระพริบ
  digitalWrite(13, HIGH); // เปิดหลอดไฟที่ขา 13
  delay(500); // หน่วงเวลา 500 milliseconds
  digitalWrite(13, LOW); // ปิดหลอดไฟที่ขา 13
  delay(500); // หน่วงเวลา 500 milliseconds
}

ros::Subscriber<std_msgs::String> sub("chatter", chatterCallback); 
// สร้างตัวแปร sub สำหรับ subscribe กับ topic ชื่อ chatter และใช้ฟังก์ชัน chatterCallback เมื่อมีข้อความใหม่

void setup() {
  pinMode(13, OUTPUT); // กำหนดให้ขา 13 เป็นขาออก
  Serial.begin(115200); // เริ่มการติดต่อสื่อสารผ่าน serial ที่ความเร็ว 115200 baud rate
  nh.initNode(); // เริ่มต้น node ใน ROS
  nh.subscribe(sub); // subscribe กับ topic ชื่อ chatter โดยใช้ตัวแปร sub
}

void loop() {
  nh.spinOnce(); // ให้ ROS จัดการการสื่อสารและเรียก callback function ที่จำเป็น
  delay(10); // หยุดการทำงานชั่วคราวเป็นเวลา 10 milliseconds เพื่อป้องกันการใช้ CPU สูงเกินไป
}
