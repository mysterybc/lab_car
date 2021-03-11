#include "dialogdetail.h"
#include "ui_dialogdetail.h"

DialogDetail::DialogDetail(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogDetail)
{
    ui->setupUi(this);
}

DialogDetail::~DialogDetail()
{
    delete ui;
}
