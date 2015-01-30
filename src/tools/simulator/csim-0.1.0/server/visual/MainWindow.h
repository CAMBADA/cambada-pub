#ifndef _MAINWINDOW_HH_
#define _MAINWINDOW_HH_

#include "ui_MainWindow.h"
#include "render/RenderWidget.hh"

#include <QLabel>

#include "Time.hh"

namespace visual {
  
  class MainWindow : public QMainWindow, private Ui::VisualAppMW {
    Q_OBJECT
    
  public:
    MainWindow(QWidget* parent = 0);
    
    void updateVisual();
    bool UserClosedWindow();
    
  public slots:
  
    void PauseSimulation();
    void SaveState();
    void RestoreState();
    
  protected:
    void closeEvent( QCloseEvent* event );
    
  private:
    bool shouldQuit;
    bool simPaused;

    QLabel* m_Status;
    QLabel* m_realTime;
    QLabel* m_SimTime;

    gazebo::Time lastRealTime;
    gazebo::Time lastSimTime;
    gazebo::Time lasUpdate;
    double  percent;

  };
  
}


#endif /* end of include guard: _MAINWINDOW_HH_ */
