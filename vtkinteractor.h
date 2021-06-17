#ifndef VTKINTERACTOR_H
#define VTKINTERACTOR_H
#include <vtkAutoInit.h>
#include "vtkRenderWindow.h"
#include <boost/shared_ptr.hpp>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointPicker.h>
#include <vtkEventQtSlotConnect.h>
#include "vtkObjectFactory.h"          //使用vtkStandardNewMacro必须要加的头文件

//定义交互样
class InteractorStyleMoveCamera : public vtkInteractorStyleTrackballCamera
{
public:
   static InteractorStyleMoveCamera* New();
   //vtk中的宏定义，帮助实现一些父类的基本函数
   vtkTypeMacro(InteractorStyleMoveCamera,vtkInteractorStyleTrackballCamera);
   virtual void OnRightButtonDown()override
   {
       vtkInteractorStyleTrackballCamera::OnRightButtonDown();
       double picked[3];
       this->Interactor->GetPicker()->GetPickPosition(picked);  //获取三维坐标
       std::cout<<picked[0]<<" "<<picked[1]<<" "<<picked[2]<<std::endl;
   }
};

vtkStandardNewMacro(InteractorStyleMoveCamera);
#endif // VTKINTERACTOR_H
