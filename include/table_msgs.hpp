//
//  table_msgs.hpp
//  serial_test
//
//  Created by brave_mac on 2023/2/14.
//
#ifndef table_msgs_hpp
#define table_msgs_hpp
#include <iostream>
#include <unistd.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>


template<typename T>
int split_int(T num);
int split_char(unsigned char*);

uint16_t do_crc_checkSum(uint8_t* buff, uint16_t len);

// ------------------------ table 28 33 ------------------------------------------
// tabel 28 basic control msg
#pragma pack(push,1)
struct table_28    //要发送的数据结构
{   //len = 83(exclude 帧头,帧wei,校验和)
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    
    uint8_t control_type;
    uint8_t device_ssr;     // devive start stop reset.
    uint8_t device_running_mode;

    unsigned char addr_rgb[24];  // how to get 24bit type?? string==24bit
    unsigned char addr_ir[24];
    unsigned char addr_lidar[24];
    unsigned char keep[4]; // 预留

    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //获取校验和
    bool is_equal(); //校验和是否一致。
};
#pragma pack(pop)

#pragma pack(push,1)
struct table_33    //要发送的数据结构
{   //len = 83(exclude 帧头,帧wei,校验和)
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
    
    
    uint8_t device_ssr;     // devive start stop reset.
    uint8_t device_running_mode;

    unsigned char addr_rgb[24];  // how to get 24bit type?? string==24bit
    unsigned char addr_ir[24];
    unsigned char addr_lidar[24];
    unsigned char keep[4]; // 预留
    
    uint8_t error_msg;  //#错误码
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    table_33& operator=(const table_28& t28);
};
#pragma pack(pop)
// ------------------------ table 28 33 ------------------------------------------

// ------------------------ table 29 34 ------------------------------------------
#pragma pack(push,1)
struct table_29    // control msg
{   //len = 83(exclude 帧头,帧wei,校验和)
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
    
    uint8_t det_isBegin;     // object detector start or stop. 00=stop 01=strat
    uint8_t det_pattern;    // object pattern
    uint8_t det_dataSource; //
    uint8_t preprocess_method;
    
    uint8_t obj_conf;   //detect confidence
    uint8_t obj_rec_conf; // recognition confidence
    
    uint8_t minimum_len; // for obj detect minimum bbox len.
    uint8_t minimum_width; // for obj detect minimum bbox w.
    
    uint8_t unknown_det_begin; //
    unsigned char keep[5]; // 预留
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    bool is_equal(); //校验和是否一致。
};
#pragma pack(pop)


#pragma pack(push,1)
struct table_34    // control msg
{   //len = 83(exclude 帧头,帧wei,校验和)
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
    
    uint8_t det_isBegin;     // object detector start or stop. 00=stop 01=strat
    uint8_t det_pattern;    // object pattern
    uint8_t det_dataSource; //
    uint8_t preprocess_method;
    
    uint8_t obj_conf;   //detect confidence
    uint8_t obj_rec_conf; // recognition confidence
    
    uint8_t minimum_len; // for obj detect minimum bbox len.
    uint8_t minimum_width; // for obj detect minimum bbox w.
    
    uint8_t unknown_det_begin; //

    unsigned char keep[5]; // 预留
    uint8_t error_msg;  //#错误码
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    table_34& operator=(const table_29& t29);
};
#pragma pack(pop)
// ------------------------ table 29 34 ------------------------------------------
// ------------------------ table 30 35 ------------------------------------------
#pragma pack(push,1)
struct table_30    // object tracking
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
    
    
    uint8_t track_isBegin;     // object detector start or stop. 00=stop 01=strat
    uint8_t track_pattern;    // object pattern
    uint8_t track_principle; //
    uint16_t track_id_manual;
    uint8_t track_conf;   //
    uint16_t track_maximum; //
    uint16_t track_free; //
    
    unsigned char keep[4]; // 预留
    
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    bool is_equal(); //校验和是否一致。
};
#pragma pack(pop)

#pragma pack(push,1)
struct table_35    // object tracking
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
    
    
    uint8_t track_isBegin;     // object detector start or stop. 00=stop 01=strat
    uint8_t track_pattern;    // object pattern
    uint8_t track_principle; //
    uint16_t track_id_manual;
    uint8_t track_conf;   //
    uint16_t track_maximum; //
    uint16_t track_free; //
    
    unsigned char keep[4]; // 预留
    uint8_t error_msg; //错误码
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    table_35& operator=(const table_30& t30);
};
#pragma pack(pop)

// ------------------------ table 30 35 ------------------------------------------
// ------------------------ table 31 36 ------------------------------------------

#pragma pack(push,1)
struct table_31    // object alart msg
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
    
    
    uint8_t alart_isBegin;     // object detector start or stop. 00=stop 01=strat
    unsigned char alart_port[24];    // object pattern  
    uint8_t alart_trigger_mode; //
    uint8_t alart_classes;  //可变长度 数据长度-（消息代码~预留）------
    // unsigned char alart_classes[512];
    uint8_t alart_threat_level;   //
    uint8_t unknownObj_alart_conf; //
    uint16_t alart_free; //
    
    unsigned char keep[4]; // 预留
    
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    bool is_equal(); //校验和是否一致。
};
#pragma pack(pop)


#pragma pack(push,1)
struct table_36    // object alart msg
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
    
    
    uint8_t alart_isBegin;     // object detector start or stop. 00=stop 01=strat
    unsigned char alart_port[24];    // object pattern
    uint8_t alart_trigger_mode; //
    uint8_t alart_classes;
    uint8_t alart_threat_level;   //
    uint8_t unknownObj_alart_conf; //
    uint16_t alart_free; //
    
    unsigned char keep[4]; // 预留
    uint8_t error_msg; //错误码
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    table_36& operator=(const table_31& t31);
};
#pragma pack(pop)

// ------------------------ table 31 36 ------------------------------------------
// ------------------------ table 32 37 ------------------------------------------
#pragma pack(push,1)
struct table_32    // object detection classes control msg
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
        
    uint8_t obj_classes;     // see appendix 2

    unsigned char keep[2]; // 预留
    
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    bool is_equal(); //校验和是否一致。
};
#pragma pack(pop)

#pragma pack(push,1)
struct table_37    // object detection classes control msg
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t control_type;
        
    uint8_t obj_classes;     // see appendix 2

    unsigned char keep[2]; // 预留
    uint8_t error_msg;
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    table_37& operator=(const table_32& t32);
};
#pragma pack(pop)


// ------------------------ table 32 37 ------------------------------------------
// ------------------------ table 38 39 ------------------------------------------
// device condition query.

#pragma pack(push,1)
struct table_38    // object detection device‘s condition query.
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
        
    uint8_t query_type;  // condition or ability query 01H 02H

    unsigned char keep[2]; // 预留
    
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
    bool is_equal(); //校验和是否一致。
};
#pragma pack(pop)


#pragma pack(push,1)
struct table_39    // object detection device‘s condition query.  return msg.
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
        
    uint8_t feedback_type;  // device condition feedback.
    unsigned char data_source[96];
    uint8_t device_stop_status;
    
    uint8_t det_pattern;
    uint8_t det_dataSource_status;
    uint8_t preprocess_method;
    uint8_t obj_conf;   //detect confidence
    uint8_t obj_rec_conf; // recognition confidence
    uint8_t minimum_len; // for obj detect minimum bbox len.
    uint8_t minimum_width; // for obj detect minimum bbox w.
    uint8_t unknown_det_status;
    
    uint8_t track_func_status;
    uint8_t track_pattern;    // object pattern
    uint8_t track_principle; //
    uint16_t track_maximum; //
    uint8_t track_conf;   //
    
    uint8_t alart_func_status;
    unsigned char alart_port[24];    // object pattern
    uint8_t alart_trigger_mode; //
    uint8_t alart_classes;
    uint8_t alart_threat_level;   //
    uint8_t unknownObj_alart_conf; //
    
    unsigned char keep[4]; // 预留
    uint8_t error_msg;
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
};
#pragma pack(pop)

#pragma pack(push,1)
struct table_40    // object detection device‘s ability query.  return msg.
{
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
        
    uint8_t feedback_type;  // condition or ability query 01H 02H

    uint8_t data_type_supported;
    uint8_t is_preprocess_method; // if set or not.
    uint8_t is_det_conf;
    uint8_t is_recog_conf;
    uint8_t is_minimumSize;
    uint8_t is_unknown;
    uint8_t is_track_conf;
    
    uint8_t is_alart_func;
    uint8_t is_3d_bbox;
    uint8_t is_recog_msg_conf;
    uint8_t is_obj_det_msg_conf;
    uint8_t is_unknown_conf;
    
    unsigned char keep[1]; // 预留
    uint8_t error_msg;
    uint16_t checkSum; //#校验和
    uint8_t tail;  //#帧尾
    
    uint16_t get_checkSum(); //成员函数不占空间
};
#pragma pack(pop)





// ------------------------ table 44  image upload msg ------------------------------------------
#pragma pack(push,1)
struct table_44    // object detection device‘s ability query.  return msg.
{
    table_44();
    ~table_44();
    const static int width   =1920;
    const static int height  =1080;
    const static int channel = 3;
    
    uint16_t get_checkSum(); //成员函数不占空间
    
    uint16_t header; //帧头
    uint16_t data_len; //数据长度
    uint16_t msg_code;  //消息代码
    uint8_t msg_class; //03H 图像信息
    
    uint32_t image_id;
    uint16_t pkg_order; //包序号
    uint8_t pkg_type;
    
    uint8_t pkg_total_num;
    
    std::unique_ptr<unsigned char[]> img_data;
//      unsigned char* img_data;  //数据，太大爆了size + 8
    
    unsigned char keep[6]; // 预留
    uint16_t checkSum; //#校验和
    uint8_t tail=0xAA;  //#帧尾
    
    
    
};
#pragma pack(pop)

// ------------------------ table 44  image upload msg ------------------------------------------



#endif /* table_msgs_hpp */





/*
 
 test for push image
 
 //
 //  main.cpp
 //  push_udi
 //
 //  Created by feifeiwei on 2023/2/18.
 //

 #include <iostream>
 #include <sys/socket.h>
 #include <arpa/inet.h>
 #include <stdio.h>
 #include <iomanip>
 #include <string.h>
 #include <unistd.h>
 #include <iostream>
 //#include "table_msgs.hpp"

 #include <opencv2/opencv.hpp>
 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <memory>

 #define PORT 18000


 cv::Mat ucharArray2Mat(unsigned char* frame_char, int width, int height, int channel)
 {
     cv::Mat image_mat;
     if (channel == 4)
         image_mat = cv::Mat(height, width, CV_8UC4, frame_char);
     if (channel == 3)
         image_mat = cv::Mat(height, width, CV_8UC3, frame_char);
     if (channel == 1)
         image_mat = cv::Mat(height, width, CV_8UC1, frame_char);
     return image_mat;
 }





 #pragma pack(push,1)
 struct table_44    // object detection device‘s ability query.  return msg.
 {
     table_44();
     ~table_44();
     const static int width   =1920;
     const static int height  =1080;
     const static int channel = 3;
     
     uint16_t get_checkSum(); //成员函数不占空间
     
     uint16_t header; //帧头
     uint16_t data_len; //数据长度
     uint16_t msg_code;  //消息代码
     uint8_t msg_class; //03H 图像信息
     
     uint32_t image_id;
     uint16_t pkg_order; //包序号
     uint8_t pkg_type;
     
     uint8_t pkg_total_num;
     
     std::unique_ptr<unsigned char[]> img_data;
 //      unsigned char* img_data;  //数据，太大爆了size + 8
     
     unsigned char keep[6]; // 预留
     uint16_t checkSum; //#校验和
     uint8_t tail=0xAA;  //#帧尾
     
     
     
 };
 #pragma pack(pop)




 table_44::table_44()
 {
 //    img_data = new unsigned char[this->width*this->height*this->channel];
     img_data = std::make_unique<unsigned char[]>(this->width*this->height*this->channel);
 }

 table_44::~table_44()
 {
 //    delete []img_data;
 //    img_data.reset();
 }



 class Socket_pushMsg
 {
 public:
     Socket_pushMsg(const char* ip_send, int port):
     ip_send(ip_send), port(port)
     {
         memset(&servaddr, 0, sizeof(servaddr)); //把servaddr内存清零
         sock = socket(AF_INET, SOCK_DGRAM, 0);
         servaddr.sin_family = AF_INET;
         servaddr.sin_port = htons(port);
         servaddr.sin_addr.s_addr = inet_addr(ip_send);
     }
     void send_msg_t44( table_44& t44, int chunk_size=1024) // -1 send once for all data,
     {
         
         int buffer_len = sizeof(table_44) + table_44::width*table_44::height*table_44::channel - 8; //数据总长度 + new
         int chunk = chunk_size; //每次发送长度‘
         unsigned char* data = new unsigned char[buffer_len];
 //        unsigned char* header = data; // make pointer point to header
         std::cout <<"buff len= " << buffer_len << std::endl;
         
         long idx = 0;
         data[idx++] = t44.header >> 8;
         data[idx++] = t44.header & 0xff;
         
         data[idx++] = t44.data_len >> 8;
         data[idx++] = t44.data_len & 0xff;
         
         data[idx++] = t44.msg_code >> 8;
         data[idx++] = t44.msg_code & 0xff;
         
         data[idx++] = t44.msg_class;
         
         data[idx++] = t44.image_id >> 24;
         data[idx++] = t44.image_id >> 16;
         data[idx++] = t44.image_id >> 8;
         data[idx++] = t44.image_id & 0xff;
  
         data[idx++] = t44.pkg_order >> 8;
         data[idx++] = t44.pkg_order & 0xff;
         
         data[idx++] = t44.pkg_type;
         data[idx++] = t44.pkg_total_num;
     
         for (int i=0; i<table_44::width*table_44::height*table_44::channel; i++) {
             data[idx++] = t44.img_data[i];
         }
         
 //        cv::Mat im = ucharArray2Mat(t44.img_data.release(), 1920, 1080, 3);
 //        imshow("im",im);
 //        cv::waitKey(0);
         
         
         for(const auto& c: t44.keep) data[idx++]  = c;
         
         data[idx++] = t44.checkSum >> 8;
         data[idx++] = t44.checkSum & 0xff;
         
         data[idx++] = t44.tail;
         
         assert(buffer_len==idx); //ensure not  missing data.

         while (buffer_len>chunk) {
             ssize_t ret = sendto(sock, data, chunk, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
             buffer_len -= chunk;
             data = data + chunk;
             assert(ret>0);

             
         }
         if(buffer_len){
             ssize_t ret = sendto(sock, data, buffer_len, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
             assert(ret>0);
         }
     }
     
     
     
     template<class T>
     void send_msg_chunk(T& table, int chunk_size=1024) //分断发送
     {
         int buffer_len = sizeof(T);
         int chunk = chunk_size; //每次发送长度
     
         unsigned char* data = reinterpret_cast<unsigned char*>(&table);
         
         while (buffer_len>chunk) {
             ssize_t ret = sendto(sock, data, chunk, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
             buffer_len -= chunk;
             data = data + chunk;
         }
         if(buffer_len){
             ssize_t ret = sendto(sock, &table, buffer_len, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
         }
     }
     
     
     template<class T>
     void send_msg(T& table) //一次性发送
     {
         ssize_t ret = sendto(sock, &table, sizeof(T), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
     }
     
     void push_image(const cv::Mat& img, int width, int height)
     {
         int optval=height*width*10; //设置缓冲区
         int optLEn=sizeof(int);
         setsockopt(sock,SOL_SOCKET,SO_RCVBUF,(char*)&optval,optLEn);

         int buf;
         unsigned char* data = img.data;
         int flag=300;
         sendto(sock,&flag,sizeof(flag),0,(struct sockaddr*)&servaddr,sizeof(servaddr));//发送帧头
         
         for(int i=0;i<height;i++)
         {
             sendto(sock,data+i*width*3,sizeof(unsigned char)*width*3,0,(struct sockaddr*)&servaddr,sizeof(servaddr));//一行一行的发送图像
         }
     }
     
     ~Socket_pushMsg()
     {
         close(sock);
     }
     
 private:
     std::string ip_send;
     int port;
     struct sockaddr_in servaddr;
     int sock; //定义socket套字，  发送消息，图像等
 };

 #pragma pack(push,1)
 struct table_28    //要发送的数据结构
 {   //len = 83(exclude 帧头,帧wei,校验和)
     uint16_t header; //帧头
     uint16_t data_len; //数据长度
     uint16_t msg_code;  //消息代码
     
     uint8_t control_type;
     uint8_t device_ssr;     // devive start stop reset.
     uint8_t device_running_mode;

     unsigned char addr_rgb[24];  // how to get 24bit type?? string==24bit
     unsigned char addr_ir[24];
     unsigned char addr_lidar[24];
     unsigned char keep[4]; // 预留

     uint16_t checkSum; //#校验和
     uint8_t tail;  //#帧尾
     
     uint16_t get_checkSum(); //获取校验和
     bool is_equal(); //校验和是否一致。
 };
 #pragma pack(pop)


 using namespace std;
 int main(int argc, const char * argv[]) {
     // insert code here...
     std::cout << "Hello, World!\n";
     
     Socket_pushMsg sp("127.0.0.1", PORT);
     
     cv::Mat im = cv::imread("/Users/feifeiwei/dog.jpg");
     
     std::cout <<im.cols <<" " << im.rows << endl;
     
     
     table_44 t44;
 //    t44.img_data = std::make_unique<unsigned char[]>(0);
 //    t44.img_data = im.data;
     unsigned char *data = new unsigned char[1920*1080*3];//im.data;
     memcpy(data, im.data, 1920*1080*3);
     
     t44.header = 0x1234;
     
     t44.img_data.reset(data);
     
     sp.send_msg_t44(t44, 1024);
     
 //    auto p = t44.img_data.release();
 //
 //    cv::Mat image_mat = cv::Mat(1080, 1920, CV_8UC3, p);//
 //
 //    cv::imshow("sss", image_mat);
 //    cv::waitKey(5000);

 //    delete p;
 //
     
     
 //    table_28 t28;
 //    t28.header = 0x1234;
     cout <<"t44.header "<<" ---" << t44.header <<" " << sizeof(table_44) <<endl;
     
     
 //    sp.send_msg<table_28>(t28);
 //    sp.send_msg_chunk(t28);
     
 //    t44.img_data.reset(nullptr);
     
     return 0;
 }

 
 
 */



