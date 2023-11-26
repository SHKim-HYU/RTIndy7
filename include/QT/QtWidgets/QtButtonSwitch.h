#pragma once

#include <QtWidgets>

class QButtonSwitch : public QWidget
{
    Q_OBJECT
        Q_DISABLE_COPY(QButtonSwitch)

public:
    enum Type
    {
        YESNO,
        ONOFF,
        BOOL,
        EMPTY
    };

public:
    explicit QButtonSwitch(QWidget* parent = nullptr, Type type = Type::ONOFF);
    ~QButtonSwitch() override;

    //-- QWidget methods
    void mousePressEvent(QMouseEvent*) override;
    void paintEvent(QPaintEvent* event) override;
    void setEnabled(bool);

    //-- Setters
    void setDuration(int);
    void setValue(bool);
    void setText(const QString& atext);

    //-- Getters
    bool value() const;

signals:
    void valueChanged(bool newvalue);

private:
    class SwitchCircle;
    class SwitchBackground;
    void _update();

private:
    bool _value;
    int  _duration;

    QLinearGradient _lg;
    QLinearGradient _lg2;
    QLinearGradient _lg_disabled;

    QColor _pencolor;
    QColor _offcolor;
    QColor _oncolor;
    int    _tol;
    int    _borderradius;

    // This order for definition is important (these widgets overlap)
    QLabel* _labeloff;
    SwitchBackground* _background;
    QLabel* _labelon;
    SwitchCircle* _circle;

    bool _enabled;

    QPropertyAnimation* __btn_move;
    QPropertyAnimation* __back_move;
};

class QButtonSwitch::SwitchBackground : public QWidget
{
    Q_OBJECT
        Q_DISABLE_COPY(SwitchBackground)

public:
    explicit SwitchBackground(QWidget* parent = nullptr, QColor color = QColor(154, 205, 50), bool rect = false);
    ~SwitchBackground() override;

    //-- QWidget methods
    void paintEvent(QPaintEvent* event) override;
    void setEnabled(bool);

private:
    bool            _rect;
    int             _borderradius;
    QColor          _color;
    QColor          _pencolor;
    QLinearGradient _lg;
    QLinearGradient _lg_disabled;

    bool _enabled;
};


class QButtonSwitch::SwitchCircle : public QWidget
{
    Q_OBJECT
        Q_DISABLE_COPY(SwitchCircle)

public:
    explicit SwitchCircle(QWidget* parent = nullptr, QColor color = QColor(255, 255, 255), bool rect = false);
    ~SwitchCircle() override;

    //-- QWidget methods
    void paintEvent(QPaintEvent* event) override;
    void setEnabled(bool);

private:
    bool            _rect;
    int             _borderradius;
    QColor          _color;
    QColor          _pencolor;
    QRadialGradient _rg;
    QLinearGradient _lg;
    QLinearGradient _lg_disabled;

    bool _enabled;
};