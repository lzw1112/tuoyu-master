#include "../include/cyrobot_monitor/dialog.h"

#include <QThread>
#include <QDebug>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

CanThread::CanThread(QObject *parent) : QObject(parent)
{
    isStop = true;
}

CanThread::~CanThread()
{

}

void CanThread::Thread_stop()
{
    isStop = true;
}

void CanThread::Thread_run()
{
    isStop = false;
}


 static   int dis_l;
 static   int dis_r;
Radar_STATE CanThread::tof_data()
{
    Radar_STATE radar_state;
    memset(&radar_state, 0, sizeof(Radar_STATE));

    radar_state.tof_l_dis = dis_l;
    radar_state.tof_r_dis = dis_r;
    return radar_state;
}

void CanThread::set_tof(int l , int r)
{
        dis_l = l;
        dis_r = r;

}


void CanThread::startThreadSlot()
{
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_filter rfilter[1];
    Radar_STATE radar_state;
    memset(&radar_state, 0, sizeof(Radar_STATE));

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr); //指定 can0 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can0 绑定
    //定义接收规则,只接收表示符等于 0x11 的报文
    //rfilter[0].can_id = 0x11;
    //rfilter[0].can_mask = CAN_SFF_MASK;
    //设置过滤规则
    //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    while (1)
    {
        if(isStop == true)
        {

            QThread::sleep(1);
            radar_state.tof_l_dis = 0;
            radar_state.tof_r_dis = 0;

                                dis_l = radar_state.tof_l_dis ;
                                dis_r = radar_state.tof_r_dis ;
                                  set_tof(dis_l , dis_r);
            emit sen_state(radar_state);
            continue;
        }

        else
        {
      
#if 1
            nbytes = read(s, &frame, sizeof(frame));

            //接收报文//显示报文
            if(nbytes > 0)
            {

                //printf("ID=0x%X DLC=%d data[0]=0x%X\n",frame.can_id,frame.can_dlc,frame.data[0]);
            //printf(“ID=0x%X DLC=%d data[0]=0x%X\n”, frame.can_id,	frame.can_dlc, frame.data[0]);
                switch (frame.can_id)
                {                        
                        case 0x343:

                                radar_state.tof_l_dis = frame.data[2];
                                radar_state.tof_r_dis = frame.data[3];

                                dis_l = radar_state.tof_l_dis ;
                                dis_r = radar_state.tof_r_dis ;
                                  set_tof(dis_l , dis_r);

                                radar_state.tof_r_state = 1;
                            break;
                        case 0x347:
                                if ((BIT_VAL(frame.data[6], 3)) == 1)//frame.data[6]右后探头状态error
                                {
                                    radar_state.tof_r_state = 1;
                                }

                                if ((BIT_VAL(frame.data[6], 2)) == 1)//frame.data[6]右中探头状态error
                                {
                                    radar_state.tof_r_mid_state = 1;
                                }

                                if ((BIT_VAL(frame.data[6], 1)) == 1)//frame.data[6]左中探头状态error
                                {
                                    radar_state.tof_l_mid_state = 1;
                                }

                                if ((BIT_VAL(frame.data[6], 0)) == 1)//frame.data[6]左后探头状态error
                                {
                                    radar_state.tof_l_state = 1;
                                }
                            break;
                        case 0x201:
 printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 0;
                            break;
                       case 0x211:
printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 1;
                            break;
                       case 0x221:
printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 2;
                            break;
                       case 0x231:
printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 3;
                            break;
                       case 0x241:
printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 4;
                            break;
                       case 0x251:
printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 5;
                            break;
                       case 0x261:
printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 6;
                            break;
                       case 0x271:
printf("[%#x]---\n",frame.can_id);
                                if ((BIT_VAL(frame.data[2], 4)) == 1)//frame.data[2]interface_error
                                {
                                    radar_state.radar_interference = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 1)) == 1)//frame.data[2]vol_error
                                {
                                    radar_state.radar_vol = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 3)) == 1)//frame.data[2]temperature_error
                                {
                                    radar_state.radar_temperture = 1;
                                }
                                if ((BIT_VAL(frame.data[2], 5)) == 1)//frame.data[2]persistent_error
                                {
                                    radar_state.radar_state = 1;
                                }
                                if (((BIT_VAL(frame.data[2], 3)) == 1) && ((BIT_VAL(frame.data[2], 2)) != 1))//frame.data[5]state_cluster
                                {
                                    radar_state.radar_dec_clu = 1;
                                }
                                if (((BIT_VAL(frame.data[5], 3)) != 1) && ((BIT_VAL(frame.data[5], 2)) == 1))//frame.data[5]state_object
                                {
                                    radar_state.radar_dec_obj = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]pow stand
                                {
                                    radar_state.radar_pow = 1;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 0))//frame.data[3]
                                {
                                    radar_state.radar_pow = 2;
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 0) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -3
                                {
                                }
                                if (((BIT_VAL(frame.data[3], 0)) == 1) && ((BIT_VAL(frame.data[3], 1)) == 1))//frame.data[3]pow -9
                                {
                                    radar_state.radar_pow = 4;
                                }
                                radar_state.radar_id = 7;
                            break;

                    default:
                        break;
                }
                emit sen_state(radar_state);
usleep(1*1000);
            }
#endif
            //QThread::sleep(1);


        }
    }
}
