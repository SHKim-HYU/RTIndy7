#pragma once

#include "QtAbstractMeter.h"

    class QThermoMeter : public QAbstractMeter
    {
        Q_OBJECT 
        public:
            QThermoMeter(QWidget *parent = 0);
        protected:
            void paintEvent(QPaintEvent *event); 	  // inherited form WidgetWithBackground
            void paintBackground(QPainter & painter); // inherited form WidgetWithBackground
            void initCoordinateSystem(QPainter & painter);
    };

