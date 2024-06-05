/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>   // เรียกใช้ library ของ ROS
#include <std_msgs/String.h>// เรียกใช้ library std_msgs type เป็น String(ข้อความ)

ros::NodeHandle  nh; //การสร้าง NodeHandle ชื่อ nh

std_msgs::String str_msg;  //สร้างตัวแปรชื่อ str_msg มารับตัวจัดการ msgs type String
ros::Publisher chatter("chatter", &str_msg); //สร้างตัวเเปรชื่อ chatter มารับความสามารถ Publisher    ชื่อตัวแปร("ชื้อ TOPIC", &ตัวแปรที่สร้างให้มาจัดการ msgs);

char hello[13] = "hello ros!";//สร้างตัวเเปรชนิด  char ให้เก็บอักขระได้ 13 ตัว hello เก็บอักขระต่อกันเป็นคำว่า  hello ros! 

void setup()
{
  nh.initNode(); // สร้าง node
  nh.advertise(chatter); //ประกาศ Publisher
}

void loop()
{
  str_msg.data = hello; //เอาตัวแปร str_msg ที่สร้างจากบรรทัดที่ 11 มารับค่าของ hello คือคำว่า hello ros!
  chatter.publish( &str_msg );//เอาตัวเเปร chatter ที่เป็น Publisher มาส่งข้อมูลไปยัง  ros โดยส่งคำว่า hello ros! ที่เก็บในตัวแปร
  nh.spinOnce();//ให้ ros ทำซ้ำ
  delay(1000);
}