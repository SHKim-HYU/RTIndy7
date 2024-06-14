
#include <assert.h>
#include "QtAbstractMeter.h"
#include "QtFunctions.h" 

QAbstractMeter::QAbstractMeter(QWidget * parent )
 : QWidgetWithBackground (parent)
{
  m_min=m_minimum=0.0;
  m_max=m_maximum=1.0;
  m_digitOffset=1.0;
  m_nominal=0.25;
  m_critical=0.75; 
} 


bool QAbstractMeter::calcMaxMin()
{
 return range(m_minimum,m_maximum,m_min,m_max,8,true); 
}

void QAbstractMeter::setValue( double val )
{
  if ( m_value != val )
  {
    m_value = val;
    update(); 
    emit valueChanged(val);
    emit valueChanged((int)val); 
  }
}

void QAbstractMeter::setValue( int val )
{
  if ( m_value != val )
  {
    m_value = val;
    update(); // Ciekawe czy tak jest lepiej ??
    // to znaczy najpierw odmalowa?a potem generowa?sygna?? 
    emit valueChanged(val);
    emit valueChanged(double(val));
  }
}

void QAbstractMeter::setMinimum(double i)
{
  if ((m_maximum - i) > 0.00001 )
  {
    m_minimum = i;
    if (calcMaxMin()) updateWithBackground();
  }
}

void QAbstractMeter::setMaximum(double i)
{
  if ( (i - m_minimum) > 0.00001 )
  {
    m_maximum = i;
    if (calcMaxMin()) updateWithBackground();
  }
}


