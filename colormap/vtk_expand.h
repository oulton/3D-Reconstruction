//
// Created by wy on 2021/8/1.
//

#ifndef PROJECT_VTK_EXPAND_H
#define PROJECT_VTK_EXPAND_H

#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkImageViewer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLight.h>
#include <vtkCamera.h>
#include <vtkLightCollection.h>
#include <vtkRenderer.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkTriangle.h>
#include <vtk3DSImporter.h>
#include <vtkDataSet.h>
#include <vtkOBJReader.h>
#include <vtkLookupTable.h>
#include <vtkAxesActor.h>
#include <vtkPolyDataNormals.h>
#include <vtkMaskPoints.h>

#include <vtkBMPWriter.h>
#include <vtkImageWriter.h>
#include <vtkJPEGWriter.h>
#include <vtkPNGWriter.h>
#include <vtkPNMWriter.h>
#include <vtkPostScriptWriter.h>
#include <vtkTIFFWriter.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <locale>
#include <string>

cv::Mat
vtk_visualization(std::string filename, int width, int height, Eigen::Matrix3d intrinsic, Eigen::Matrix4d extrinsic) {

    double fx = intrinsic(0, 0);
    double fy = intrinsic(1, 1);
    double cx = intrinsic(0, 2);
    double cy = intrinsic(1, 2);
    Eigen::Matrix3d r = extrinsic.block(0, 0, 3, 3);
    Eigen::Vector3d t = extrinsic.block(0, 3, 3, 1);

    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(t[0], t[1], t[2]);
    // 根据相机外矩阵，设置焦点位置。设置完这两项成像平面已经唯一确定。
    camera->SetFocalPoint(r(0, 2) + t[0], r(1, 2) + t[1], r(2, 2) + t[2]);
    // 根据相机外矩阵，设置成像y正方向。是否有负号取决于原始的相机坐标系中，y是朝向相机上方（正）还是下方（负）。
    camera->SetViewUp(-r(0, 1), -r(1, 1), -r(2, 1));
    // 计算视角。注意这里我的焦距值focus是负数，所以前面加负号。VTK中ViewAngle用角度表示。
    double viewAngle = 2 * atan((height / 2) / fy) * 180 / CV_PI;
    camera->SetViewAngle(viewAngle);
    // 计算窗口中心。xh和yh分别是光学中心相对于传感器中心的偏移量。注意正方向规定的差异导致的负号。建议正负号都试一下。
    double windowCenter_x = -(cx - 0.5 * width) / (width / 2);
    double windowCenter_y = (cy - 0.5 * height) / (height / 2);
    camera->SetWindowCenter(windowCenter_x, windowCenter_y);

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetAmbient(0.1);
    actor->GetProperty()->SetSpecular(0.2);
    actor->GetProperty()->SetDiffuse(0.6);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->GetProperty()->ShadingOn();

    vtkSmartPointer<vtkAxesActor> actor2 = vtkSmartPointer<vtkAxesActor>::New();
    actor2->SetPosition(0, 0, 0);
    actor2->SetTotalLength(10, 10, 10);
    actor2->SetShaftType(0);
    actor2->SetAxisLabels(0);
    actor2->SetCylinderRadius(0.02);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    //renderer->AddActor(actor2);//显示坐标系
    renderer->SetBackground(1.0, 1.0, 1.0);
    //renderer->AddLight(light);//将灯光加入渲染器
    renderer->SetActiveCamera(camera);
    renderer->ResetCameraClippingRange(); //不加这一行可能会成像不完整。

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(width, height);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderWindow->Render();

    //screen  rgb
    vtkSmartPointer<vtkWindowToImageFilter> wti = vtkSmartPointer<vtkWindowToImageFilter>::New();
    wti->SetInput(renderWindow);
    wti->SetInputBufferTypeToRGB();
    wti->ReadFrontBufferOff();
    wti->Update();
    vtkSmartPointer<vtkPNGWriter> PNGWriter = vtkSmartPointer<vtkPNGWriter>::New();
    PNGWriter->SetFileName("./color.png");
    PNGWriter->SetInputConnection(wti->GetOutputPort());
    PNGWriter->Write();

    //depth
    cv::Mat depthImage = cv::Mat(cv::Size(width, height), CV_16UC1, cv::Scalar(0));
    float *data = renderWindow->GetZbufferData(0, 0, width - 1, height - 1);

    double wPos[4];
    // 用于将世界坐标系转化为相机坐标系下深度值，外参矩阵的第三行。
    Eigen::Matrix4d extrinsic_ = extrinsic.inverse();
    double m20 = extrinsic_(2, 0);
    double m21 = extrinsic_(2, 1);
    double m22 = extrinsic_(2, 2);
    double m23 = extrinsic_(2, 3);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // 获取ZBuffer对应数值
            float depth = data[y * width + x];
            // 为1时表示无穷远，无深度
            if (depth == 1) {
                depthImage.at<ushort>(y, x) = 0;
            } else {
                //将zBuffer转化为世界坐标系点坐标
                vtkInteractorObserver::ComputeDisplayToWorld(renderer, x, y, depth, wPos);
                // 将世界坐标系点坐标，转化为相机坐标系下Z，即深度
                float z = wPos[0] * m20 + wPos[1] * m21 + wPos[2] * m22 + m23;
                // 将米单位转化为毫米单位uint16_t进行存储
                depthImage.at<ushort>(y, x) = round(z * 1000);
            }
        }
    }
    delete data;
    //renderWindowInteractor->Start();
    // 由于vtkRenderWindow的x、y方向与相机的不一致，需要进行flip。
    cv::flip(depthImage, depthImage, 0);
    // 保存深度图
    cv::imwrite("./depth.png", depthImage);

    return depthImage;
}


void vtk_visualization(std::string filename, int width, int height) {

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();

    reader->SetFileName(filename.c_str());
    reader->Update();
    mapper->SetInputConnection(reader->GetOutputPort());

//    string::size_type pos = filename.find(".");
//    if (pos == filename.npos)
//    {
//        cerr<<"filename format isn't match\n"<<endl;
//    }
//
//    if ( filename.substr(pos)=="obj"){
//        vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
//        reader->SetFileName(filename.c_str());
//        reader->Update();
//        mapper->SetInputConnection(reader->GetOutputPort());
//    }else if( filename.substr(pos)=="ply") {
//        vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
//        reader->SetFileName(filename.c_str());
//        reader->Update();
//        mapper->SetInputConnection(reader->GetOutputPort());
//    }

//    double fx = intrinsic(0, 0);
//    double fy = intrinsic(1, 1);
//    double cx = intrinsic(0, 2);
//    double cy = intrinsic(1, 2);
//    Eigen::Matrix3d r = extrinsic.block(0, 0, 3, 3);
//    Eigen::Vector3d t = extrinsic.block(0, 3, 3, 1);
//
//    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
//    camera->SetPosition(t[0], t[1], t[2]);
//    // 根据相机外矩阵，设置焦点位置。设置完这两项成像平面已经唯一确定。
//    camera->SetFocalPoint(r(0, 2) + t[0], r(1, 2) + t[1], r(2, 2) + t[2]);
//    // 根据相机外矩阵，设置成像y正方向。是否有负号取决于原始的相机坐标系中，y是朝向相机上方（正）还是下方（负）。
//    camera->SetViewUp(-r(0, 1), -r(1, 1), -r(2, 1));
//    // 计算视角。注意这里我的焦距值focus是负数，所以前面加负号。VTK中ViewAngle用角度表示。
//    double viewAngle = 2 * atan((height / 2) / fy) * 180 / CV_PI;
//    camera->SetViewAngle(viewAngle);
//    // 计算窗口中心。xh和yh分别是光学中心相对于传感器中心的偏移量。注意正方向规定的差异导致的负号。建议正负号都试一下。
//    double windowCenter_x = -(cx - 0.5 * width) / (width / 2);
//    double windowCenter_y = (cy - 0.5 * height) / (height / 2);
//    camera->SetWindowCenter(windowCenter_x, windowCenter_y);


    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetAmbient(0.1);
    actor->GetProperty()->SetSpecular(0.2);
    actor->GetProperty()->SetDiffuse(0.6);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->GetProperty()->ShadingOn();

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(1.0, 1.0, 1.0);
    //renderer->AddLight(light);//将灯光加入渲染器
    //renderer->SetActiveCamera(camera);
    renderer->ResetCameraClippingRange(); //不加这一行可能会成像不完整。

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(width, height);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderWindow->Render();
    renderWindowInteractor->Start();
}


#endif //PROJECT_VTK_EXPAND_H
