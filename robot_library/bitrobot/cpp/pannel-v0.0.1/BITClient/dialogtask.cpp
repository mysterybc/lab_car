#include "dialogtask.h"
#include "ui_dialogtask.h"

DialogTask::DialogTask(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogTask)
{
    ui->setupUi(this);
    ui->tbInfo->document()->setMaximumBlockCount(1000);
}

DialogTask::~DialogTask()
{
    delete ui;
}

void DialogTask::on_btnClear_clicked() {
    clear();
}
void DialogTask::clear() {
    ui->tbInfo->clear();
}
void DialogTask::newInfo(const QString& info) {
    ui->tbInfo->append(info);
}
