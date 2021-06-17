#include "mainwindow.h"
#include "ui_mainwindow.h"
#include"QVTKWidget.h"
#include"vtkinteractor.h"
#include"start.h"
#include <QDir>   //用于打印当前路径
#include <QFileDialog>
#include <QMessageBox>
#include <QFile>
#include <QFileDevice>
#include <QTextStream>
#include <QDebug>
#include<vtkSTLReader.h>
#include <vtkKdTree.h>
#include <vtkDecimatePro.h>

#include<unistd.h>   //线程相关的
#include <pthread.h>
#include"MathTool.h"

//显示
//模拟旋转后的点云

//只在一个文件中使用的全局变量用static表示 ，表明该变量的作用域只在当前文件
static bool isChangePoint;      //标记是改变所选的点云，还是创建新的点云
static bool isPicking;          //标记是否选中了点云中的某一点，从而开始侦听鼠标事件
static bool isMoving;           //标记是否按下m，从而开始移动

static unsigned long set_index;           //选中点所在的点簇下标
static unsigned long set_inside_index;    //选中点所在点簇内的点的下标

static vtkSmartPointer<vtkKdTree> tree_target;
//static vtkSmartPointer<vtkActor> referenActor;  //用于显示的点集，相当于PCL中的某个特定名字的点云
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //初始化
    tree_target = vtkSmartPointer<vtkKdTree>::New();
    winBackColor = vtkSmartPointer<vtkNamedColors>::New();
    style = vtkSmartPointer<MyInteractorStyle>::New();
    //初始化界面内容与全局变量
    ModePointSelect=-1;     //默认选点方式，什么都不做
    ModeDeviation="kepp";   //设置偏移点的方法

    isChangePoint =false;
    isPicking = false;
    isMoving = false;
    cloud_target = vtkSmartPointer<vtkPoints>::New();
    cloud_choosed = vtkSmartPointer<vtkPoints>::New();
    new_cloud_choosed = vtkSmartPointer<vtkPoints>::New();

    RegistrationOrder_1 = "NONE\nNONE\n0.0\nNONE\n";
    RegistrationOrder_2 = "NONE\nNONE\n0.0\nNONE\n";


    Tough=0;
    Fine=0;

    FILENAME = "/home/saran/Code/QT/06016Pthread/source/";
    //窗口控件初始化,设定参数不可写
    ui->deviation_set_tough->setReadOnly(true);
    ui->deviation_set_fine->setReadOnly(true);

    printf("载入界面");

}

MainWindow::~MainWindow()
{
    delete ui;
}
//鼠标左键
vtkStandardNewMacro(MyInteractorStyle);
void MyInteractorStyle::OnLeftButtonDown()
{

//  std::cout << "Picking pixel: " << this->Interactor->GetEventPosition()[0]
//            << " " << this->Interactor->GetEventPosition()[1] << std::endl;
  this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
                                      this->Interactor->GetEventPosition()[1],
                                      0, // always zero.
                                      this->Interactor->GetRenderWindow()
                                          ->GetRenderers()
                                          ->GetFirstRenderer());
  //对应的三维坐标
  double picked[3];
  this->Interactor->GetPicker()->GetPickPosition(picked);
  //std::cout << "Picked value: " << picked[0] << " " << picked[1] << " "<< picked[2] << std::endl;


  vtkSmartPointer<vtkIdList> index = vtkSmartPointer<vtkIdList>::New();
  tree_target->FindClosestNPoints(1, picked, index);
  std::copy(picked,picked+3,pick_point->point);  //记录选择的点
  pick_point->index = index->GetId(0);           //记录选择点的下标
  //改变选点
  switch(ModePointSelect)
  {
  //选中点，就删除之后怎么增加
   case 1:
  {
      vtkSmartPointer<vtkPoints> cloud_kicked = vtkSmartPointer<vtkPoints>::New();
      cloud_kicked->DeepCopy(cloud_choosed) ;//鼠标选中的簇，主要用来显示,需要使用深拷贝，避免cloud_choosed被修改
      cloud_kicked->InsertNextPoint(picked);

      UpdatePointsShow(cloud_kicked,referenActor);

      break;
  }
  //选中点簇和点
  //(只有选点簇时的点时黄色的，确定之后点都变成红色)
   case 2:
  {
      //isPicking = true;             //传递鼠标移动事件的信号，不需要触发，直接在本函数内部注册鼠标事件
      //复制之前的选中点集；
      new_cloud_choosed->DeepCopy(cloud_choosed);   //深拷贝，对其修改不影响原点云

      //首先将随机点定位到离其最近的点，
      vtkSmartPointer<vtkKdTree>tree_choosed = vtkSmartPointer<vtkKdTree>::New();
      tree_choosed->BuildLocatorFromPoints(new_cloud_choosed);

      vtkSmartPointer<vtkIdList> index_choose = vtkSmartPointer<vtkIdList>::New();
      tree_choosed->FindClosestNPoints(1, picked, index);

      double temp_current_point[3];
      float current_point[3];
      new_cloud_choosed->GetPoint(index->GetId(0),temp_current_point);
      std::copy(temp_current_point, temp_current_point + 3, current_point);   //从double到float的转换
      choosed_index = index->GetId(0);
      ReallyDeletePoint(new_cloud_choosed,index->GetId(0));//删除cloud中的该点
      //查找选中点的下标
      bool hasfind = false;
      for(unsigned long i=0;i<SelectPoints.size();i++)
      {
          for(unsigned long j = 0;j<SelectPoints[i].size();j++)
          {
              if(isSamePoint(SelectPoints[i][j].point,current_point))
              {
                   set_index = i;         //记录点簇所在点的下标
                   set_inside_index = j;  //记录簇内当前点的下标
                   hasfind = true;
                   break;
              }
          }
          if(hasfind)
          {
              break;
          }
      }
       //更新旧点云和选中的黄点云
      vtkSmartPointer<vtkPoints> cloud_kicked = vtkSmartPointer<vtkPoints>::New();
      cloud_kicked->InsertNextPoint(current_point);//鼠标选中的簇，主要用来显示
      //绿色显示
      UpdatePointsShow(cloud_kicked,chooseActor);

      //显示去掉一个点后的refrence点云
      //红色显示
      UpdatePointsShow(new_cloud_choosed,referenActor);
      break;
  }
  case 3:   //change:选中点重选为其他店(黄色)
  {
      vtkSmartPointer<vtkPoints> cloud_kicked = vtkSmartPointer<vtkPoints>::New();
      cloud_kicked->InsertNextPoint(picked);//鼠标选中的簇，主要用来显示

      //绿色显示
      UpdatePointsShow(cloud_kicked,chooseActor);

      break;
  }
  case 4:  //和1一样，只是键盘操作不一样
  {
      vtkSmartPointer<vtkPoints> cloud_kicked = vtkSmartPointer<vtkPoints>::New();
      cloud_kicked->DeepCopy(cloud_choosed) ;//鼠标选中的簇，主要用来显示,需要使用深拷贝，避免cloud_choosed被修改
      cloud_kicked->InsertNextPoint(picked);

      UpdatePointsShow(cloud_kicked,referenActor);

       break;

  }
  default:
      break;
  }

  // Forward events
  vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}
//键盘事件
void MyInteractorStyle::OnKeyPress()
{
  // Get the keypress
  vtkRenderWindowInteractor* rwi = this->Interactor;
  std::string key = rwi->GetKeySym();

  // Output the key that was pressed
  std::cout << "Pressed " << key << std::endl;

  if(ModePointSelect==1)   //需要开始增加参考点
  {
      if (key == "space" )  //Return space
      {
          IsNext=1;  //开始进行精配准
      };
      if (key == "Return" )  //Return Enter    加上选择的点
      {
          cout << pick_point->point[0] << " " << pick_point->point[1] << " " << pick_point->point[2] << endl;
          OneArea.push_back(*pick_point);    //使用map的原因就是为了防止同一个点重复被加入
          cloud_choosed->InsertNextPoint(pick_point->point);
          std::cout << "存入该点" << endl;
      }
      if (key == "Escape" )  //Return Esc   一个簇已经选择完毕
      {
          SelectPoints.push_back(OneArea);
          OneArea.clear();
          std::cout << "存入线段" << endl;
      }
  }
  if(ModePointSelect==3)     //正在移动点时
  {
      if (key == "Return")  //Return Enter    保存移动修改后的点
      {
          SelectPoints[set_index][set_inside_index] = *pick_point;//替换修改后的点

          cloud_choosed->SetPoint(choosed_index,pick_point->point);//在点簇点云中替换修改后的点
          //移除表示移动中的黄点
          vtkSmartPointer<vtkPoints> cloud_kicked = vtkSmartPointer<vtkPoints>::New();
          std::cout<<cloud_kicked->GetNumberOfPoints()<<std::endl;
          UpdatePointsShow(cloud_kicked,chooseActor);

          //显示其中一个点被替换后的reference点云
          UpdatePointsShow(cloud_choosed,referenActor);
           std::cout << "改变该点" << endl;
           //变回选取模式
           ModePointSelect=2;
      }
  }
  if(ModePointSelect==4)
  {
      if (key == "Return")  //Return Enter    加上选择的点
      {
          SelectPoints[set_index].push_back(*pick_point);    //在选择点中增加
          cloud_choosed->InsertNextPoint(pick_point->point); //显示点云增加
          //显示增加一个点后的refrence点云
          UpdatePointsShow(cloud_choosed,referenActor);
          std::cout << "新增该点" << endl;
          //变回选取模式
          ModePointSelect=2;
      }
  }
     // std::cout<<event.getKeySym()<<std::endl;
  // Forward events
  vtkInteractorStyleTrackballCamera::OnKeyPress();
}
//通过过滤要删除的ID将点复制到另一个临时vtkPoints,然后将其浅拷贝到原始的vtkPoints：
//这样的删除方法不会影响之前指向的points
void MyInteractorStyle::ReallyDeletePoint(vtkSmartPointer<vtkPoints> points, vtkIdType id)
{
    vtkSmartPointer<vtkPoints> newPoints = vtkSmartPointer<vtkPoints>::New();

      for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
      {
        if (i != id)
        {
          double p[3];
          points->GetPoint(i, p);
          newPoints->InsertNextPoint(p);
        }
      }
    //浅拷贝，只是更新引用计数
    points->ShallowCopy(newPoints);
}

//打开pcd模拟旋转文件
//尝试开辟线程，让其在后台读取文件：主线程更改界面，子线程做运算。
void* pthrd_func(void* arg)
{

}
void MainWindow::on_open_source_clicked()
{
    QString fileName;
     fileName = QFileDialog::getOpenFileName(this,"open","/home/saran/Data/测试用例/北大一院");                                      //打开所有文件
    std::string file_name = fileName.toStdString();
    std::cout<<"文件路径:"<<file_name<<std::endl;

    //source
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(file_name.c_str());
    reader->Update();
    vtkSmartPointer<vtkPolyData> stlData = reader->GetOutput();


   //mapper
     vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
     mapper->SetInputData(stlData);

     vtkSmartPointer<vtkPointPicker> pointPicker =  vtkSmartPointer<vtkPointPicker>::New();
   //actor
     vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
     actor->SetMapper(mapper);

   //render
     vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
      renderer->AddActor(actor);
      renderer->SetBackground(winBackColor->GetColor3d( "DeepSkyBlue" ).GetData() );
   //renderwindow
     vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
     renderWindow->AddRenderer(renderer);
     renderWindow->SetWindowName("PointPicker");

     ui->CloudViewer->SetRenderWindow(renderWindow);
     ui->CloudViewer->GetInteractor()->SetPicker(pointPicker);      //这句话加不加都一样
     ui->CloudViewer->GetInteractor()->SetInteractorStyle(style);

     style->SetCurrentRenderer(renderer);

     ui->CloudViewer->setEnabled(true);

     // Render and interact
     renderWindow->Render();
     ui->CloudViewer->update ();


     //存储数据,注意时浅拷贝，仅仅是指针指向新的points
     std::cout<<stlData->GetNumberOfPoints()<<"个点"<<std::endl;
     cloud_target  = stlData->GetPoints();
     tree_target->BuildLocatorFromPoints(cloud_target);
     stl_target = stlData;
     file_name.erase(file_name.begin()+file_name.find("hip_left.stl"), file_name.end());
     OPENROUTE = file_name;


    //在Linux线程下读取文件
//    pthread_t id;
//    int ret;
//    ret = pthread_create(&id,NULL,pthrd_func,(void*)this);
//    pthread_detach(id);
//    //在Qthread线程下

}

//读取之前记录的参考点   ReadMap()
//每次打开都重置点云的显示和存储的vector
void MainWindow::on_open_txt_clicked()
{
    //还原读取留下的数据
    SelectPoints.clear();
    OneArea.clear();
    cloud_choosed->Reset();

    QString fileName;
    //fileName = QFileDialog::getOpenFileName(this,tr("Open File"),tr(""),tr("Text File (*.txt)"));   //只显示指定后缀的文件
    fileName = QFileDialog::getOpenFileName(this,"open",FILENAME.data());                                      //打开所有文件
    if(fileName == "")                  //如果用户直接关闭了文件浏览对话框，那么文件名就为空值，直接返回
    {
        return;
    }
    else
    {
        FILE *txt_point = nullptr;
        std::string filename = fileName.toStdString();
        std::cout<<filename<<std::endl;                //每次第一次调用文件名时，前面都会自动加上载入界面，操作原字符串没用
        //filename.erase (filename.begin(),filename.begin()+4);
        //std::cout<<filename<<std::endl;
        txt_point = fopen(filename.data(), "rb");
        if (!txt_point)
        {
            QMessageBox::warning(this,tr("错误"),tr("打开文件失败"));
            return;
        }
        int index;
        float temp_point[3];   //静态数组，在栈上申请的内存，生命周期到之后会自动释放
        reference* temp_reference = new reference();
        while (!feof(txt_point))                             //这个读取方法会导致多读一次最后一行
        {
            fscanf(txt_point, "%d %f %f %f ", &index, &temp_point[0], &temp_point[1], &temp_point[2]);
            if (index == -1)
            {
                SelectPoints.push_back(OneArea);
                OneArea.clear();
            }
            else
            {
                std::copy(temp_point,temp_point+3,temp_reference->point); //深拷贝，不能仅仅复制指针
                temp_reference->index = index;
                OneArea.push_back(*temp_reference);
                cloud_choosed->InsertNextPoint(temp_point);    //存入参考点云
            }
        }
        fclose(txt_point);    //关闭该文本
        delete temp_reference;

        //更新选择的参考点
        style->UpdatePointsShow(cloud_choosed,style->referenActor);

        //在嵌入窗口显示，但是：1、回调函数不能正常使用 2、窗口大小不会自适应
         ui->CloudViewer->update ();
    }

}
//vtk信号对应的槽函数，已经可以使用了
void MainWindow::slot_clicked(vtkObject* obj, unsigned long, void*, void*)
{
   vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::SafeDownCast(obj);
   //double pick[3];
   //iren->GetPicker()->GetPickPosition(pick);
   int position[2];
   iren->GetEventPosition(position);
   std::cout<<"点击了鼠标左键,发出信号"<<position[0]<<" "<<position[1]<<std::endl;

}
//采用vtk将所有pcd的点集读取后显示，vtk也有自己的

void MainWindow::on_Translation_Estimate_currentIndexChanged(const QString &arg1)
{
    //std::string modex =
}

void MainWindow::on_Start_clicked()
{
    //在开始配准前，首先读取当前界面中的设定
    //配准指令
    RegistrationOrder_1 =  ui->Selected_Mode_1->toPlainText().toStdString();
    RegistrationOrder_2 =  ui->Selected_Mode_2->toPlainText().toStdString();
    //偏移设定
    if(ui->deviation_keep->isChecked()==true)
    {
        ModeDeviation ="keep";
    }
    else if(ui->deviation_random->isChecked()==true)
    {
        ModeDeviation = "random";
    }
    else if(ui->deviation_set->isChecked()==true)
    {
        ModeDeviation = "set";
        Tough = ui->deviation_set_tough->text().toInt();
        Fine = ui->deviation_set_fine->text().toInt();
    }
    //旋转矩阵设定
    XCAngle = ui->xc_angle->text().toFloat();
    YCAngle = ui->yc_angle->text().toFloat();
    ZCAngle = ui->zc_angle->text().toFloat();

    XRAngle = ui->xr_angle->text().toFloat();
    YRAngle = ui->yr_angle->text().toFloat();
    ZRAngle = ui->zr_angle->text().toFloat();

    IsCenterRotation = ui->center_rotation->isChecked();

    XRDistance = ui->xr_distance->text().toFloat();
    YRDistance = ui->yr_distance->text().toFloat();
    ZRDistance = ui->zr_distance->text().toFloat();

    SampleRate = ui->sample_rate->text().toDouble();

    RegistrationSatrt* main_start = new RegistrationSatrt(cloud_target);   //构建配准类，并传入模型
    main_start->start();
    delete main_start;
    main_start = NULL;

    //初始化全局变量
    IsNext = 0;
    IsClose = false;
}

//存储当前的参考点WriteMap()
void MainWindow::on_save_txt_clicked()
{
    QFileDialog fileDialog;
    QString str = fileDialog.getSaveFileName(this,tr("Open File"),FILENAME.data(),tr("Text File(*.txt)"));
    if(str == "")
    {
        return;
    }
    ofstream fa;
    std::cout<<str.toStdString().data()<<std::endl;
    fa.open(str.toStdString().data(), ios::out);
    for (unsigned long i = 0; i < SelectPoints.size(); i++)
    {
        std::vector<reference>::iterator it;
        for (it = SelectPoints.at(i).begin(); it != SelectPoints.at(i).end(); it++)
        {
            fa << it->index << " " << it->point[0] << " " << it->point[1] << " " << it->point[2] << endl;
        }
        fa << -1 << " " << -1 << " " << -1 << " " << -1 << endl;   //作为一个簇结束的标记
    }
    fa.close();
    QMessageBox::information(this,"保存文件","保存文件成功",QMessageBox::Ok);
}

void MainWindow::on_Mode_Add_ONE_clicked()
{
    std::string ModeTrans;       //矩阵估计的方法
    std::string ModePara;        //权值函数
    float ParameterValue;     //权值函数对应的参数
    std::string ModeCorr;        //匹配点对的方法

    //读取当前界面的设定
    ModeTrans = ui->Translation_Estimate->currentText().toStdString();
    if(ModeTrans == "NONE")
    {
        //弹出提示对话框
    }
    ModePara = ui->Paraments->currentText().toStdString();  //只有鲁棒函数可以为NONE
    if(ModePara!="NONE")
    {
         ParameterValue = ui->Paraments_Value->text().toFloat();
    }
    else
    {
       ParameterValue = 0.0;
    }
    ModeCorr = ui->Correspond->currentText().toStdString();
    //将当前界面的设定作为指令
    std::stringstream ss;
    ss<<ModeTrans<<std::endl;
    ss<<ModePara<<std::endl;
    ss<<ParameterValue<<std::endl;
    ss<<ModeCorr<<std::endl;

    //在界面中显示-----------------------------------------------------------------------------------------
    ui->Selected_Mode_1->setText(QString::fromStdString(std::string(ss.str())));
}

void MainWindow::on_MODE_ADD_TWO_clicked()
{
    std::string ModeTrans;       //矩阵估计的方法
    std::string ModePara;        //权值函数
    float ParameterValue;     //权值函数对应的参数
    std::string ModeCorr;        //匹配点对的方法

    //读取当前界面的设定
    ModeTrans = ui->Translation_Estimate->currentText().toStdString();
    ModePara = ui->Paraments->currentText().toStdString();
    if(ModePara!="NONE")
    {
         ParameterValue = ui->Paraments_Value->text().toFloat();
    }
    else
    {
        ParameterValue = 0.0;
    }
    ModeCorr = ui->Correspond->currentText().toStdString();
    //将当前界面的设定作为指令
    std::stringstream ss;
    ss<<ModeTrans<<std::endl;
    ss<<ModePara<<std::endl;
    ss<<ParameterValue<<std::endl;
    ss<<ModeCorr<<std::endl;

    ui->Selected_Mode_2->setText(QString::fromStdString(std::string(ss.str())));
}


//重新选取特征点
void MainWindow::on_insert_point_clicked()
{
    ModePointSelect = 1;     //将键盘的交互模式改为创建新的txt
}

//选中待改变的点
//点集select按钮、change、add、delete操作执行完成之后，ModePointSelect的模式变为2
void MainWindow::on_select_point_clicked()
{
    ModePointSelect = 2;
}
//更换已经确定的refrence点
void MainWindow::on_change_point_clicked()
{
    ModePointSelect = 3;   //开始更换选中的点

}

void MainWindow::on_add_point_clicked()
{
    ModePointSelect = 4;   //将键盘的交互模式改为插入点
}

//点击之后会一致删除点
void MainWindow::on_delete_point_clicked()
{
      ui->CloudViewer->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(style->chooseActor);
      ui->CloudViewer->GetRenderWindow()->Render();
      SelectPoints[set_index].erase(SelectPoints[set_index].begin()+set_inside_index); //删除在vector中的该点
      cloud_choosed->DeepCopy(new_cloud_choosed);    //指针指向删除后的点集,注意指针直接赋值的后果！！！
}


OpenSourceThread::OpenSourceThread(QObject *parent) : QThread(parent)
{
    tt = (MainWindow*)parent;
}
void OpenSourceThread::run()
{
   pthrd_func(tt);
   emit isDone();    //发出线程结束信号
}
