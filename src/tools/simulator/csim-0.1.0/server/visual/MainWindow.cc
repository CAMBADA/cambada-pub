

#include <QtGui>
#include <QTime>

#include "MainWindow.h"

#if defined(signals) && defined(QOBJECTDEFS_H) && \
  !defined(QT_MOC_CPP)
#  undef signals
#  define signals signals
#endif

#include "World.hh"
#include "Simulator.hh"

#if defined(signals) && defined(QOBJECTDEFS_H) && \
  !defined(QT_MOC_CPP)
#  undef signals
// Restore the macro definition of "signals", as it was
// defined by Qt's <qobjectdefs.h>.
#  define signals protected
#endif

using namespace visual;
using namespace gazebo;

MainWindow::MainWindow( QWidget* parent ) : QMainWindow(parent) {
  setupUi(this);

  // Simulation actions...
  connect(action_Quit, SIGNAL(triggered()), this, SLOT(close()));
  connect(action_Pause, SIGNAL(triggered()), this, SLOT(PauseSimulation()));

  connect(actionSaveState,    SIGNAL(triggered()), this, SLOT(SaveState()));
  connect(actionRestoreState, SIGNAL(triggered()), this, SLOT(RestoreState()));
  
  this->m_Status    = new QLabel(this);
  this->m_realTime  = new QLabel(this);
  this->m_SimTime   = new QLabel(this);

  this->sb->addPermanentWidget(this->m_realTime);
  this->sb->addPermanentWidget(this->m_SimTime);
  this->sb->addPermanentWidget(this->m_Status);

  this->shouldQuit = false;

  connect( this->OGL, SIGNAL(actionInfo(QString, int)), this->sb, SLOT(showMessage(QString, int)) );

}

void MainWindow::closeEvent( QCloseEvent* event ){
  event->accept();
  this->shouldQuit = true;
}

void MainWindow::updateVisual(){
  this->OGL->updateGL();



  if (  (Simulator::Instance()->GetRealTime() - this->lasUpdate ).Double() < 0.05 )
      return;

  double real   = 0;
  double sim    = 0;


  Time realTime = Simulator::Instance()->GetRealTime();
  Time simTime  = Simulator::Instance()->GetSimTime();

  this->percent = ((simTime - this->lastSimTime) / (realTime - this->lastRealTime)).Double();
  this->lastSimTime = simTime;
  this->lastRealTime= realTime;

  this->lasUpdate = realTime;

  real= realTime.Double();
  sim = simTime.Double();

  QString realStr;
  QString simStr;
  QString infoStr;

  if (sim > 31536000){
      sim/= 31536000;
      simStr = QString("S %1 (dys)")
               .arg( sim, 0, 'f', 1 );
  } else if ( sim > 86400 ){
      sim/= 86400;
      simStr = QString("S %1 (dys)")
               .arg( sim, 0, 'f', 1 );
  } else if ( sim > 3600 ){
      sim/= 3600;
      simStr = QString("S %1 (hrs)")
               .arg( sim, 0, 'f', 1 );
  } else if ( sim > 60) {
      sim/= 60;
      simStr = QString("S %1 (min)")
               .arg( sim, 0, 'f', 1 );
  } else {
      simStr = QString("S %1 (sec)")
               .arg( sim, 0, 'f', 1 );
  }

  if (real > 31536000){
      real/= 31536000;
      realStr = QString("R %1 (dys)")
               .arg( real, 0, 'f', 1 );
  } else if ( real > 86400 ){
      real/= 86400;
      realStr = QString("R %1 (dys)")
               .arg( real, 0, 'f', 1 );
  } else if ( real > 3600 ){
      real/= 3600;
      realStr = QString("R %1 (hrs)")
               .arg( real, 0, 'f', 1 );
  } else if ( real > 60) {
      real/= 60;
      realStr = QString("R %1 (min)")
               .arg( real, 0, 'f', 1 );
  } else {
      realStr = QString("R %1 (sec)")
               .arg( real, 0, 'f', 1 );
  }

  if ( this->simPaused )
      infoStr = QString("[Sim Paused]");
  else
      infoStr = QString("[%1x]").arg( this->percent , 0, 'f', 2 );


  this->m_SimTime->setText( simStr );
  this->m_realTime->setText( realStr );
  this->m_Status->setText( infoStr );
}

bool MainWindow::UserClosedWindow(){
  return this->shouldQuit;
}


void MainWindow::PauseSimulation(){

    this->simPaused = ! this->simPaused;
    Simulator::Instance()->SetPaused( this->simPaused );

    if ( this->simPaused ){
        this->action_Pause->setText( QString("Un&Pause") );
    }else{
        this->action_Pause->setText( QString("&Pause") );
    }

}

void MainWindow::SaveState(){
  World::Instance()->SaveState();
  this->sb->showMessage(QString("World state saved"), 5000 );
}

void MainWindow::RestoreState(){
  World::Instance()->RestoreState();
  this->sb->showMessage(QString("World state restored"), 5000 );
}



