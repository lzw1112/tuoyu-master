#ifndef DIALOG_H
#define DIALOG_H

#include <QObject>


#ifndef BIT_VAL
#define BIT_VAL(value, idx) (((value) >> (idx)) & 1)
#endif

#ifndef BIT_SET
#define BIT_SET(value, idx) (value &= ~(1ULL << idx) )
#endif

typedef struct
{
    int tof_l_state;
    int tof_l_mid_state;
    int tof_r_mid_state;
    int tof_r_state;
    int tof_l_dis;
    int tof_r_dis;
    int radar_temperture;
    int radar_state;
    int radar_vol;
    int radar_interference;
    int radar_dec_clu;
    int radar_dec_obj;
    int radar_pow;
    int radar_id;
} Radar_STATE;


class CanThread : public QObject
{
    Q_OBJECT
public:
    explicit CanThread(QObject *parent = nullptr);
    ~CanThread();
    void Thread_stop();
    void Thread_run();
    Radar_STATE tof_data();
public slots:
    void startThreadSlot();

private:
    volatile bool isStop;

    void set_tof(int, int);
signals:
    void sen_state(Radar_STATE radar_state);

};

#endif // DIALOG_H
