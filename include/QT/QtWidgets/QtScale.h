#pragma once

#include <QWidget>

class QScale : public QWidget
{
    Q_OBJECT

public:
    QScale(QWidget *parent = 0);
    ~QScale();

    double minimum() const;
    double maximum() const;

    double value() const;

    void setLabelsVisible(bool);
    bool isLabelsVisible() const;

    void setScaleVisible(bool);
    bool isScaleVisible() const;

    void setBorderWidth(int);
    int borderWidth() const;

    void setLabelsFormat(char format, int precision = -1);

    double majorStepSize() const;
    int minorStepCount() const;

    void setInvertedAppearance(bool invert);
    bool invertedAppearance() const;

    Qt::Orientations orientations() const;

//    QSize sizeHint() const;
//    QSize minimumSizeHint() const;

public Q_SLOTS:
    void setMinimum(double);
    void setMaximum(double);
    void setRange(double min, double max);

    void setValue(double);

    void setMajorStepSize(double);
    void setMinorStepCount(int);

    void setOrientations(Qt::Orientations);

protected:
//    bool event(QEvent *e);
    void resizeEvent(QResizeEvent *re);
    void paintEvent(QPaintEvent *pe);

private:
    double m_minimum;
    double m_maximum;

    double m_value;

    bool m_labelsVisible;
    bool m_scaleVisible;

    int m_borderWidth;

    char m_labelsFormat;
    int m_labelsPrecision;

    double m_majorStepSize;
    int m_minorStepCount;

    bool m_invertedAppearance;

    Qt::Orientations m_orientations;

    QPainter *painter;
    QString labelSample;

    double pi;

    void updateLabelSample();
};

