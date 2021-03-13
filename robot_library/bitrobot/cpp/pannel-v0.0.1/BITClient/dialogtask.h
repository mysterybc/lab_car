#ifndef DIALOGTASK_H
#define DIALOGTASK_H

#include <QDialog>

namespace Ui {
class DialogTask;
}

class DialogTask : public QDialog
{
    Q_OBJECT

public:
    explicit DialogTask(QWidget *parent = nullptr);
    ~DialogTask();

public slots:
    void newInfo(const QString& name);
    void clear();

private slots:
    void on_btnClear_clicked();

private:
    Ui::DialogTask *ui;
};

#endif // DIALOGTASK_H
