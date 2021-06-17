#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QtWidgets/QMainWindow>
#include<QThread>

#include <vtkAutoInit.h>
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointPicker.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSphereSource.h>
#include <vtkNamedColors.h>
#include"start.h"
//这个只能控制鼠标的动作
class MyInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static MyInteractorStyle* New();  //对于继承自vtkObjectBase的类，必须覆盖New()来生成一个实际类的对象，而不是得到一个vtkObjectBase对象
  vtkTypeMacro(MyInteractorStyle, vtkInteractorStyleTrackballCamera)  //通过该宏，覆盖vtk的基本函数
   MyInteractorStyle()
   {
       referenActor = vtkSmartPointer<vtkActor>::New();  //通过颜色标记不同意义的点云，便于指定更新
       referenActor->GetProperty()->SetColor(1.0,0,0);
       chooseActor = vtkSmartPointer<vtkActor>::New();
       chooseActor->GetProperty()->SetColor(0,1.0,0);
       pick_point = new reference();
   }
   void SetActor(vtkSmartPointer<vtkActor> SourceActor)
   {
       referenActor = SourceActor;
   }
  virtual void OnLeftButtonDown() override;
  virtual void OnKeyPress() override;
   //@source::用于更新显示的点集
   //@Goal:待更新的actor
   //@color:更新后点集的颜色
   void UpdatePointsShow(vtkSmartPointer<vtkPoints> Source,vtkSmartPointer<vtkActor>& Goal)
   {

       vtkIdType Source_number = Source->GetNumberOfPoints();
       vtkSmartPointer<vtkActorCollection> actorCollection = vtkSmartPointer<vtkActorCollection>::New();
       actorCollection = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActors();
       //将actorCollection的current指针重置为top
       actorCollection->InitTraversal();
       //删除对应颜色的旧点云
       if(actorCollection->GetNumberOfItems()>1)
       {
           std::cout<<"referenActor size"<<referenActor->GetProperty()->GetPointSize()<<std::endl;
           vtkSmartPointer<vtkActor> temp = vtkSmartPointer<vtkActor>::New();
           int actor_index;
           //红色(1,0,0),绿色(0,1,0);
           for (actor_index=0;actor_index<actorCollection->GetNumberOfItems();actor_index++)  //覆盖已有同色点云
           {
               temp = actorCollection->GetNextActor();
               if(Goal->GetProperty()->GetColor()[0]==temp->GetProperty()->GetColor()[0]&&Goal->GetProperty()->GetColor()[1]==temp->GetProperty()->GetColor()[1])
               {
                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(temp);  //要先删除，再改变点，再添加
                   break;
               }
           }
       }
       //若待修改的点数为0，说明只是需要删除当前点，则直接绘制之后退出
       if(Source_number==0)
       {
           this->Interactor->Render();
           return;
       }
       //添加对应颜色的新点云
       vtkSmartPointer<vtkPolyData> tempPD  = vtkSmartPointer<vtkPolyData>::New();
       tempPD->SetPoints(Source);
       vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
       vtkIdType pid[1];
         for(vtkIdType i=0;i< Source->GetNumberOfPoints();i++)
         {
                 pid[0] = i;
                vertices->InsertNextCell(1,pid);
         }
       tempPD->SetVerts(vertices);
       vtkSmartPointer<vtkPolyDataMapper> tempMP  = vtkSmartPointer<vtkPolyDataMapper>::New();
       tempMP->SetInputData(tempPD);

       vtkSmartPointer<vtkActor> tempActor = vtkSmartPointer<vtkActor>::New();
       tempActor->SetMapper(tempMP);
       tempActor->GetProperty()->SetColor(Goal->GetProperty()->GetColor()); //根据Goal的颜色，重新上色
       tempActor->GetProperty()->SetPointSize(10);  //scale和pointsize默认都是1
       tempActor->GetProperty()->GetDiffuse();
       Goal = tempActor;
       this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(Goal);
       this->Interactor->Render();
   }
  vtkSmartPointer<vtkActor> referenActor;
  vtkSmartPointer<vtkActor> chooseActor;
private:
  reference* pick_point;//所选点的信息
  int choosed_index;    //所选点在点簇点云中的下标
  void ReallyDeletePoint(vtkSmartPointer<vtkPoints> points, vtkIdType id);
};
class vtkEventQtSlotConnect;

namespace Ui {
class MainWindow;
}
class MainWindow;
class OpenSourceThread : public QThread
{
    Q_OBJECT
public:
    explicit OpenSourceThread(QObject *parent = 0);
 protected:
    //不能直接调用
    void run();
    MainWindow* tt;
signals:
    void isDone();   //注册线程任务完成的信号
public slots:
};
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
public slots:
    void slot_clicked(vtkObject*, unsigned long, void*, void*);
    friend void* pthrd_func(void* arg);    //将线程的回调函数设置为界面的友元函数，从而可以访问该类的非公有变量
private slots:
    void on_open_source_clicked();

    void on_change_point_clicked();

    void on_open_txt_clicked();

    void on_Translation_Estimate_currentIndexChanged(const QString &arg1);

    void on_Start_clicked();

    void on_save_txt_clicked();

    void on_MODE_ADD_TWO_clicked();

    void on_Mode_Add_ONE_clicked();

    void on_insert_point_clicked();


    void on_select_point_clicked();

    void on_delete_point_clicked();

    void on_add_point_clicked();


private:
    Ui::MainWindow *ui;
    vtkSmartPointer<vtkNamedColors> winBackColor;
    vtkSmartPointer<MyInteractorStyle> style;
};

#endif // MAINWINDOW_H
