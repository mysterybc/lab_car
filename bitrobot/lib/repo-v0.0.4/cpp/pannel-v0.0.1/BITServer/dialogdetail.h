#ifndef DIALOGDETAIL_H
#define DIALOGDETAIL_H

#include <QDialog>

namespace Ui {
class DialogDetail;
}

class DialogDetail : public QDialog
{
    Q_OBJECT

public:
    explicit DialogDetail(QWidget *parent = nullptr);
    ~DialogDetail();

private:
    Ui::DialogDetail *ui;
};

#endif // DIALOGDETAIL_H
