#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QHBoxLayout>
#include <QLineEdit>

#include <opencv2/opencv.hpp>

static const size_t n_sensors = 10 ;
static const size_t max_sensor_val = 255 ;


class ImageCanvas: public QWidget {
    Q_OBJECT
public:
    ImageCanvas(uint n_rows, uint n_cols, uint cell_width, QWidget *parent) ;

    void updateImage(uint count, const cv::Mat &im) ;

public slots:
    void setMinVal(int val) ;
    void setMaxVal(int val) ;

protected:
    void paintEvent(QPaintEvent *event) ;

private:

    const uint img_w_ = 8, img_h_ = 8 ;
    const uint padding_ = 16 ;

    cv::Mat images_[10] ;
    uint n_rows_ = 3, n_cols_ = 4 ;
    uint cell_sz_ ;
    uint min_val_ = 0 , max_val_ = max_sensor_val ;
};

class RecordNode ;
class CaptureDashboard: public QWidget
{
    Q_OBJECT

public:

    
    CaptureDashboard(QWidget *parent = 0);

    void updateImage(uint c, const cv::Mat &im) ;
    void setRecordNode(std::shared_ptr<RecordNode> node) ;
    
private:

    QHBoxLayout *createRangeSpinBoxes() ;
    QHBoxLayout *createSensorCombos() ;
    QComboBox *createSensorCombo() ;
    
    ImageCanvas *image_canvas_ ;
    QSpinBox *min_val_, *max_val_ ;
    QComboBox *sensor_1_, *sensor_2_, *sensor_3_ ;
    QLineEdit *prefix_ ;
    std::shared_ptr<RecordNode> record_node_ ;
    bool recording_ = false ;
    
 };