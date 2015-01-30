#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <string>

#include "Field.hh"

#include <QGLWidget>
#include <QtOpenGL>
#include <QPainter>

namespace gazebo {
  class Model;
}


class RenderWidget : public QGLWidget
{

    Q_OBJECT

public:

    enum ViewMode { kFreeView = 0, kTopView, kBallView, kRobotView }; 

    RenderWidget(QWidget* parent);
    void resizeGL(int width, int height);
    
    void PrintInfo(const std::string info);

protected:
    void initializeGL();
    void paintGL();
    //void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    
    void keyPressEvent(QKeyEvent* event);
    
    void timerEvent(QTimerEvent *event);

signals:
    void actionInfo(const QString& ainfo, int timeout = 0);

private:

    void DrawField();
    void HandleView();
    void drawInstructions(QPainter *painter, const QString& text);
    bool prefix(const char *pre, const char *str);

    csim::Field* field;
    gazebo::Model* selectedModel;
    
    int w,h;
    float viewRatio;
    
    float hfL;  // Half Field Length
    float hfW;  // Half Field Width
    float lR;   // length ratio
    float wR;   // width ratio
    
    int mx,my; 	  // mouse position
    float fx, fy; // field positions
    int mode;		  // mouse button bits

    bool printHelp;
    
    bool dragMode;
    bool rotateMode;
    bool speedMode;
    
    int viewmode;
    int toggleMoveBall;
    int watchThisRobot;
    
    std::string infoStr;
    std::string persistInfo;
    int lastTimer;
    
    float camsmooth[3];
    float camlooksmooth[3];

};

#endif // GLWIDGET_H
