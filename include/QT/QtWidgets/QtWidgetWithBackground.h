#pragma once

#include <QWidget>

class QPixmap;

class QWidgetWithBackground : public QWidget
{
	Q_OBJECT
  public:
     QWidgetWithBackground(QWidget * parent = 0);
     ~QWidgetWithBackground ();

     void  drawBackground ();
     void  updateWithBackground ();    
     bool doRepaintBackground(); 
     
  protected:

    void repaintBackground();
    virtual void  paintBackground (QPainter & painer) = 0;
    
  protected:
     QPixmap * m_pixmap;
     bool m_modified;
};


