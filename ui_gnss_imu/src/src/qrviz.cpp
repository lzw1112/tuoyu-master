#include "../include/cyrobot_monitor/qrviz.hpp"

QRviz::QRviz(QVBoxLayout *layout,QString node_name)
{

    this->layout=layout;
    this->nodename=node_name;


    //创建rviz容器
    render_panel_=new rviz::RenderPanel;
    //向layout添加widget
    layout->addWidget(render_panel_);
    //初始化rviz控制对象
    manager_=new rviz::VisualizationManager(render_panel_);
    ROS_ASSERT(manager_!=NULL);
    //获取当前rviz控制对象的 tool控制对象
    tool_manager_=manager_->getToolManager();
    ROS_ASSERT(tool_manager_!=NULL);
   //初始化camera 这行代码实现放大 缩小 平移等操作
    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->initialize();
    tool_manager_->initialize();
    manager_->removeAllDisplays();

}
QRviz::~QRviz()
{
    if (layout != nullptr && render_panel_ != nullptr)
    {
        layout->removeWidget(render_panel_);
    }

    if (render_panel_ != nullptr) delete render_panel_;
    if (manager_ != nullptr) delete manager_;

    if (current_tool != nullptr) current_tool = nullptr;
    if (tool_manager_ != nullptr) tool_manager_ = nullptr;

    ROS_INFO("RVIZ is shutdown");
}
    rviz::Display* RobotModel_=NULL;
//显示robotModel
  void QRviz::Display_RobotModel(bool enable)
  {

      if(RobotModel_==NULL)
      {
          RobotModel_=manager_->createDisplay("rviz/RobotModel","Qrviz RobotModel",enable);
      }
      else{
          delete RobotModel_;
          RobotModel_=manager_->createDisplay("rviz/RobotModel","Qrviz RobotModel",enable);
      }
  }
//显示grid
void QRviz::Display_Grid(bool enable,QString Reference_frame,int Plan_Cell_count,QColor color)
{
    if(grid_==NULL)
    {
        grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );
        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue("Billboards");
        grid_->subProp( "Color" )->setValue(color);
        grid_->subProp( "Reference Frame" )->setValue(Reference_frame);
        grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);

    }
    else{
        delete grid_;
        grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );
        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue("Billboards");
        grid_->subProp( "Color" )->setValue(color);
        grid_->subProp( "Reference Frame" )->setValue(Reference_frame);
        grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);
    }
    grid_->setEnabled(enable);
    manager_->startUpdate();
}

///
/// \brief 获取Rviz Display树状图
///
void QRviz::GetDisplayTreeModel()
{
    rviz::PropertyTreeModel *rvizmodel = manager_->getDisplayTreeModel();
    QAbstractItemModel *model = rvizmodel;
    emit ReturnModelSignal(model);
}

///
/// \brief Rviz Display的初始化与设置
/// \param ClassID
/// \param namevalue
///
void QRviz::DisplayInit(QString ClassID, bool enabled, QMap<QString, QVariant> namevalue)
{
    int num = GetDisplayNum(ClassID);
    if (num == -1)
    {
        rviz::Display *rvizDisplay = manager_->createDisplay(ClassID, ClassID, true);
        ROS_ASSERT(rvizDisplay != nullptr);
        num = GetDisplayNum(ClassID);
    }
    if (!namevalue.empty())
    {
        QMap<QString, QVariant>::iterator it;
        for (it = namevalue.begin(); it != namevalue.end(); it++)
        {
            display_group_->getDisplayAt(num)->subProp(it.key())->setValue(it.value());
        }
    }
    display_group_->getDisplayAt(num)->setEnabled(enabled);
    manager_->startUpdate();
}
void QRviz::DisplayInit(QString ClassID, QString name, bool enabled, QMap<QString, QVariant> namevalue)
{
    int num = GetDisplayNum(ClassID, name);
    if (num == -1)
    {
        rviz::Display *rvizDisplay = manager_->createDisplay(ClassID, name, true);
        ROS_ASSERT(rvizDisplay != nullptr);
        num = GetDisplayNum(ClassID, name);
    }
    if (!namevalue.empty())
    {
        QMap<QString, QVariant>::iterator it;
        for (it = namevalue.begin(); it != namevalue.end(); it++)
        {
            display_group_->getDisplayAt(num)->subProp(it.key())->setValue(it.value());
        }
    }
    display_group_->getDisplayAt(num)->setEnabled(enabled);
    manager_->startUpdate();
}

///
/// \brief 删除Display
/// \param name
///
void QRviz::RemoveDisplay(QString name)
{
    int num = GetDisplayNumName(name);
    if (num == -1)
    {
        return ;
    }
    delete display_group_->getDisplayAt(num);
//    rvizDisplays_.removeAt(num);
}
void QRviz::RemoveDisplay(QString ClassID, QString name)
{
    int num = GetDisplayNum(ClassID, name);
    if (num == -1)
    {
        return ;
    }
    delete display_group_->getDisplayAt(num);
//    rvizDisplays_.removeAt(num);
}

///
/// \brief 重命名Display
/// \param oldname
/// \param newname
///
void QRviz::RenameDisplay(QString oldname, QString newname)
{
    int num = GetDisplayNumName(oldname);
    if (num == -1)
    {
        return ;
    }
    display_group_->getDisplayAt(num)->setName(newname);
}
///
/// \brief 导出 RVIZ Display 配置
/// \param path
///
void QRviz::OutDisplaySet(QString path)
{
    if (!path.isEmpty())
    {
        if (manager_ == nullptr)
        {
            return;
        }
        rviz::Config con;
        manager_->save(con);
        rviz::YamlConfigWriter yamlconfigwriter;
        yamlconfigwriter.writeFile(con, path);
    }
}

///
/// \brief 导入 RVIZ Display 配置
/// \param path
///
void QRviz::ReadDisplaySet(QString path)
{
    if (!path.isEmpty())
    {
        if (manager_ == nullptr)
        {
            return;
        }
        rviz::YamlConfigReader yamlconfigreader;
        rviz::Config con;
        yamlconfigreader.readFile(con, path);
        manager_->load(con);
    }
}
///
/// \brief 根据Display的ClassID(和name)获得Display的序号
/// \param ClassID
/// \return
///
int QRviz::GetDisplayNum(QString ClassID)
{
    int num = -1;
    for (int i = 0; i < display_group_->numDisplays(); i++)
    {
        if (display_group_->getDisplayAt(i) != nullptr)
        {
            if (ClassID == display_group_->getDisplayAt(i)->getClassId())
            {
                num = i;
                break;
            }
        }
    }
    return num;
}
int QRviz::GetDisplayNum(QString ClassID, QString name)
{
    int num = -1;
    for (int i = 0; i < display_group_->numDisplays(); i++)
    {
        if (display_group_->getDisplayAt(i) != nullptr)
        {
            if (ClassID == display_group_->getDisplayAt(i)->getClassId() && name == display_group_->getDisplayAt(i)->getName())
            {
                num = i;
                break;
            }
        }
    }
    return num;
}
int QRviz::GetDisplayNumName(QString name)
{
    int num = -1;
    for (int i = 0; i < display_group_->numDisplays(); i++)
    {
        if (display_group_->getDisplayAt(i) != nullptr)
        {
            if (name == display_group_->getDisplayAt(i)->getName())
            {
                num = i;
                break;
            }
        }
    }
    return num;
}
//显示map
void QRviz::Display_Map(bool enable,QString topic,double Alpha,QString Color_Scheme)
{
    if(!enable&&map_)
    {
        map_->setEnabled(false);
        return ;
    }
    if(map_==NULL)
    {
        map_=manager_->createDisplay("rviz/Map","QMap",true);
        ROS_ASSERT(map_);
        map_->subProp("Topic")->setValue(topic);
        map_->subProp("Alpha")->setValue(Alpha);
        map_->subProp("Color Scheme")->setValue(Color_Scheme);

    }
    else{
         ROS_ASSERT(map_);
         qDebug()<<"asdasdasd:"<<topic<<Alpha;

        delete map_;
        map_=manager_->createDisplay("rviz/Map","QMap",true);
        ROS_ASSERT(map_);
        map_->subProp("Topic")->setValue(topic);
        map_->subProp("Alpha")->setValue(Alpha);
        map_->subProp("Color Scheme")->setValue(Color_Scheme);
    }

    map_->setEnabled(enable);
    manager_->startUpdate();
}
//显示激光雷达
void QRviz::Display_LaserScan(bool enable,QString topic)
{
    if(laser_==NULL)
    {
        laser_=manager_->createDisplay("rviz/LaserScan","QLaser",enable);
        ROS_ASSERT(laser_);
        laser_->subProp("Topic")->setValue(topic);
    }
    else{
        delete laser_;
        laser_=manager_->createDisplay("rviz/LaserScan","QLaser",enable);
        ROS_ASSERT(laser_);
        laser_->subProp("Topic")->setValue(topic);
    }
    qDebug()<<"topic:"<<topic;
    laser_->setEnabled(enable);
    manager_->startUpdate();
}
//设置全局显示
 void QRviz::SetGlobalOptions(QString frame_name,QColor backColor,int frame_rate)
 {
     manager_->setFixedFrame(frame_name);
     manager_->setProperty("Background Color",backColor);
     manager_->setProperty("Frame Rate",frame_rate);
     manager_->startUpdate();
 }

// "rviz/MoveCamera";
// "rviz/Interact";
// "rviz/Select";
// "rviz/SetInitialPose";
// "rviz/SetGoal";
 //设置机器人导航初始位置
 void QRviz::Set_Pos()
 {
     //获取设置Pos的工具
     //添加工具

     current_tool= tool_manager_->addTool("rviz/SetInitialPose");
     //设置当前使用的工具为SetInitialPose（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );
     manager_->startUpdate();

//     tool_manager_->setCurrentTool()

 }
 //设置机器人导航目标点
 void QRviz::Set_Goal()
 {
     //添加工具
     current_tool= tool_manager_->addTool("rviz/SetGoal");
     //设置goal的话题
     rviz::Property* pro= current_tool->getPropertyContainer();
     pro->subProp("Topic")->setValue("/move_base_simple/goal");
     //设置当前frame
     manager_->setFixedFrame("map");
     //设置当前使用的工具为SetGoal（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );

     manager_->startUpdate();

 }
 void QRviz::Set_MoveCamera()
 {
     //获取设置Pos的工具
     //添加工具

     current_tool= tool_manager_->addTool("rviz/MoveCamera");
     //设置当前使用的工具为SetInitialPose（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );
     manager_->startUpdate();
 }
 void QRviz::Set_Select()
 {
     //获取设置Pos的工具
     //添加工具

     current_tool= tool_manager_->addTool("rviz/Select");
     //设置当前使用的工具为SetInitialPose（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );
     manager_->startUpdate();
 }
 //显示tf坐标变换
 void QRviz::Display_TF(bool enable)
 {
     if(TF_){delete TF_;TF_=NULL;}
     TF_=manager_->createDisplay("rviz/TF","QTF",enable);
 }
 //显示导航相关
 void QRviz::Display_Navigate(bool enable,QString Global_topic,QString Global_planner,QString Local_topic,QString Local_planner)
 {
    if(Navigate_localmap) {delete Navigate_localmap; Navigate_localmap=NULL;}
    if(Navigate_localplanner) {delete Navigate_localplanner; Navigate_localplanner=NULL;}
    if(Navigate_globalmap) {delete Navigate_globalmap; Navigate_globalmap=NULL;}
    if(Navigate_globalplanner) {delete Navigate_globalplanner; Navigate_globalplanner=NULL;}
    //local map
    Navigate_localmap=manager_->createDisplay("rviz/Map","Qlocalmap",enable);
    Navigate_localmap->subProp("Topic")->setValue(Local_topic);
    Navigate_localmap->subProp("Color Scheme")->setValue("costmap");
    Navigate_localplanner=manager_->createDisplay("rviz/Path","QlocalPath",enable);
    Navigate_localplanner->subProp("Topic")->setValue(Local_planner);
    Navigate_localplanner->subProp("Color")->setValue(QColor(0,12,255));
    //global map
    Navigate_globalmap=manager_->createDisplay("rviz/Map","QGlobalmap",enable);
    Navigate_globalmap->subProp("Topic")->setValue(Global_topic);
    Navigate_globalmap->subProp("Color Scheme")->setValue("costmap");
    Navigate_globalplanner=manager_->createDisplay("rviz/Path","QGlobalpath",enable);
    Navigate_globalplanner->subProp("Topic")->setValue(Global_planner);
    Navigate_globalplanner->subProp("Color")->setValue(QColor(255,0,0));
    //更新画面显示
    manager_->startUpdate();

 }
 void QRviz::addTool( rviz::Tool* )
 {

 }
void QRviz::createDisplay(QString display_name,QString topic_name)
{


}
void QRviz::run()
{

}
