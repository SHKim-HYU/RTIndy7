
#include "qt_run.h"

void* qt_run(void* param) {
    int argc;
    char** argv;
    QApplication a(argc, argv);
    // style our application with custom dark style
    QApplication::setStyle(new DarkStyle);

    // create frameless window (and set windowState or title)
    FramelessWindow framelessWindow;

    // create our mainwindow instance
    MainWindow *mainWindow = new MainWindow;
    // add the mainwindow to our custom frameless window
    framelessWindow.resize(1600,600);
    framelessWindow.setContent(mainWindow);
    framelessWindow.show();
    a.exec();
    return NULL;
}

