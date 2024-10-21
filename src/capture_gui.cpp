#include "capture_gui.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QPainter>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QDebug>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "cv_bridge/cv_bridge.h"

#include <fstream>


using namespace std ;

class CaptureNode : public rclcpp::Node
{
public:
    CaptureNode(CaptureDashboard *dashboard,  const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): 
    dashboard_(dashboard), rclcpp::Node("capture_node", options) {
        
        
    }

    void setup() {
        image_transport_.reset(new image_transport::ImageTransport(shared_from_this())) ;

        image_transport::TransportHints hints(this);

        for( uint i=0 ; i<10 ; i++ ) {
            ostringstream s_strm ;

            s_strm << "sensor_" << i+1 << "_image" ;
            string topic  = s_strm.str() ;

            image_sub_[i] = std::make_shared<image_transport::Subscriber>(
                image_transport_->subscribe(topic, 1, std::bind(&CaptureNode::frameCallback, this, std::placeholders::_1, i))
            );

        }
       
    }
    
private:
    void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg, uint i) {
        cv::Mat img = cv_bridge::toCvShare(msg, "mono8")->image ;
         
        QMetaObject::invokeMethod(dashboard_, [=]() {
            dashboard_->updateImage(i, img);
        }, Qt::ConnectionType::QueuedConnection);
        
    }
    
private:
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    std::shared_ptr<image_transport::Subscriber> image_sub_[10];

    std::mutex frame_mutex_;
    std::atomic<bool> frame_ready_{false};
    uint count = 0 ;

    CaptureDashboard *dashboard_ ;
};

class ToFData
{
public:
    sensor_msgs::msg::Image::ConstPtr image_[3];

    ToFData(const sensor_msgs::msg::Image::ConstPtr &s1,
            const sensor_msgs::msg::Image::ConstPtr &s2,
            const sensor_msgs::msg::Image::ConstPtr &s3)
    {
        image_[0] = s1;
        image_[1] = s2;
        image_[2] = s3;
    }
};



class RecordNode : public rclcpp::Node
{
public:
    RecordNode(CaptureDashboard *dashboard,  const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): 
    dashboard_(dashboard), rclcpp::Node("record_node", options) {
        
        
    }

    void setup() {
        image_transport_.reset(new image_transport::ImageTransport(shared_from_this())) ;
    }

    void start(const string &topic1, const string &topic2, const string &topic3) {
        sync_.reset(new Synchronizer(SyncPolicy(1), image_sub_[0], image_sub_[1], image_sub_[2]));
        sync_->registerCallback(std::bind(&RecordNode::frameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        image_transport::TransportHints hints(this);

        image_sub_[0].subscribe(this, topic1, hints.getTransport(), rmw_qos_profile_default);
        image_sub_[1].subscribe(this, topic2, hints.getTransport(), rmw_qos_profile_default);
        image_sub_[2].subscribe(this, topic3, hints.getTransport(), rmw_qos_profile_default);
    }

    void stop(const std::string &prefix) {
        image_sub_[0].unsubscribe() ;
        image_sub_[1].unsubscribe() ;
        image_sub_[2].unsubscribe() ;

        sync_.reset() ;

        writeData(prefix) ;

        dataset_.clear() ;
    }
    
private:

    std::vector<ToFData> dataset_;

    void writeData(const std::string &prefix) {
    for ( uint channel = 0 ; channel < 3 ; channel ++ ) {
        std::ostringstream f_strm ;
        f_strm << prefix << channel << ".csv" ;
        auto filename = f_strm.str() ;
        ofstream ostrm(filename) ;

        for( uint frame = 0 ; frame < dataset_.size() ; frame++ ) {
            const auto &data = dataset_[frame] ;
           for( uint count = 0 ; count < 64 ; count ++ ) {
                const auto &msg = data.image_[channel] ;
                if ( count > 0 ) ostrm << ',' ;
                ostrm << (uint)(msg->data[count]) ;
           }
           ostrm << endl ;
        }
    }


}

   void frameCallback(sensor_msgs::msg::Image::ConstSharedPtr sensor1Msg,
                                     sensor_msgs::msg::Image::ConstSharedPtr sensor2Msg,
                                     sensor_msgs::msg::Image::ConstSharedPtr sensor3Msg
                                     )
{
    dataset_.emplace_back(sensor1Msg, sensor2Msg, sensor3Msg);
}
    
private:

    using SyncPolicy = typename message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    std::unique_ptr<Synchronizer> sync_;
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::SubscriberFilter image_sub_[3];

    std::mutex frame_mutex_;
    std::atomic<bool> frame_ready_{false};
    uint count = 0 ;

    CaptureDashboard *dashboard_ ;
};

ImageCanvas::ImageCanvas(uint n_rows, uint n_cols, uint cell_width, QWidget *parent): 
    n_rows_(n_rows), n_cols_(n_cols), cell_sz_(cell_width), QWidget(parent) {
        uint w = padding_ + n_cols_ * ( img_w_ * cell_width + padding_ ) ;
        uint h = padding_ + n_rows_ * ( img_h_ * cell_width + padding_ ) ;
    setMinimumSize(w, h) ;
     QPalette pal = palette();
    
    pal.setColor(QPalette::Background, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
}

static cv::Vec3b clr_map[] = {  {0, 0, 255},     // red
                                {0, 165, 255},   // orange
                                {0, 255, 255},   // yellow
                                {255, 255, 0}, // cyan
                                {255, 0, 0},   // blue
                                {128, 64, 64} };   // violet

constexpr uint n_colors = sizeof(clr_map)/sizeof(cv::Vec3b) ;

QRgb colorMap(int val, uint minVal, uint maxVal) {
    float v ;
    if ( val <= minVal ) return qRgb(0, 0, 0) ;
    else if ( val >= maxVal ) return qRgb(0, 0, 0) ;
    else v = ( val - minVal )/float( maxVal - minVal ) ;

    uint cell = std::floor(v * ( n_colors - 1)) ;
    float h = v * (n_colors - 1) - cell ;
    const auto &c0 = clr_map[cell] ;
    const auto &c1 = clr_map[cell+1] ;

    uint b = c0[0] *  (1.0f - h) + c1[0] * h ;
    uint g = c0[1] *  (1.0f - h) + c1[1] * h ;
    uint r = c0[2] *  (1.0f - h) + c1[2] * h ;

    return qRgb(r, g, b) ;
}

void ImageCanvas::paintEvent(QPaintEvent *event) {
    QPainter painter(this);

    QFont font = painter.font() ;
    font.setPointSize(18);
    painter.setFont(font);
    
    for ( uint r=0 ; r<n_rows_ ; r++ ) {
        for( uint c=0 ; c<n_cols_ ; c++ ) {
            uint index = r * n_cols_ + c ;
            if ( index >= 10 ) continue ;
            const cv::Mat &img = images_[index] ;
            if ( !img.data ) continue ;

            uint offset_x = padding_ + c * ( img_w_ * cell_sz_ + padding_ ) ;
            uint offset_y = padding_ + r * ( img_w_ * cell_sz_ + padding_ ) ;

            for( uint i=0 ; i<8 ; i++ ) {
                for( uint j=0 ; j<8 ; j++ ) {
                    uint val = img.at<uchar>(i, j) ;
                    QBrush brush;
                    QColor clr(colorMap(val, min_val_, max_val_));

                    brush.setStyle(Qt::SolidPattern);
                    brush.setColor(clr);

                    painter.setPen(Qt::NoPen);
                    painter.setBrush(brush);

                    QRect r(offset_x + j*cell_sz_, offset_y + i*cell_sz_, cell_sz_, cell_sz_) ;
                    painter.drawRect(r) ;
                }
            }

            painter.setPen(Qt::white);
            painter.drawText(offset_x, offset_y + cell_sz_, QString("%1").arg(index+1)) ;
            
        }
    }
    
}

void ImageCanvas::updateImage(uint index, const cv::Mat &im) {
    images_[index] = im ;
    update() ;
}

void ImageCanvas::setMinVal(int v) {
    min_val_ = v ;
    update() ;
}

void ImageCanvas::setMaxVal(int v) {
    max_val_ = v ;
    update() ;
}

CaptureDashboard::CaptureDashboard(QWidget *parent):
    QWidget(parent)
{
    QVBoxLayout *l = new QVBoxLayout() ;

    image_canvas_ = new ImageCanvas(3, 4, 24, this) ;

    l->addWidget(image_canvas_) ;
    l->addLayout(createRangeSpinBoxes()) ;
    l->addLayout(createSensorCombos()) ;
    l->addSpacing(12) ;

    QHBoxLayout *buttons = new QHBoxLayout() ;

    QHBoxLayout *prefix_layout = new QHBoxLayout() ;
    QLabel *prefix_label = new QLabel("Prefix") ;
    prefix_ = new QLineEdit() ;
    prefix_layout->addWidget(prefix_label) ;
    prefix_layout->addWidget(prefix_) ;
    prefix_layout->addStretch() ;

    QPushButton *start_stop_btn = new QPushButton("Start") ;
    start_stop_btn->grabShortcut(QKeySequence(Qt::Key_Space));
    start_stop_btn->setDefault(true) ;

    buttons->addLayout(prefix_layout) ;
    buttons->addWidget(start_stop_btn) ;

    QObject::connect(start_stop_btn, &QPushButton::clicked, this, [this, start_stop_btn]() {
        if ( recording_ ) {
            record_node_->stop(prefix_->text().toStdString()) ;
            start_stop_btn->setText("Start") ;
            recording_ = false ;
        }
        else {
            record_node_->start(
                QString("sensor_%1_image").arg(sensor_1_->currentData().toInt() + 1).toStdString(),
                QString("sensor_%1_image").arg(sensor_2_->currentData().toInt() + 1).toStdString(),
                QString("sensor_%1_image").arg(sensor_3_->currentData().toInt() + 1).toStdString()
            ) ;

            start_stop_btn->setText("Stop") ;
            recording_ = true ;
        }
    }) ;
    
    
    l->addLayout(buttons) ;


    l->addStretch() ;

    setLayout(l) ;

   

    
}

QHBoxLayout *CaptureDashboard::createRangeSpinBoxes() {
    QLabel *min_val_label = new QLabel(tr("Minimum value ")) ;
    min_val_ = new QSpinBox() ;
    min_val_->setRange(0, 255);
    min_val_->setSingleStep(1);
    min_val_->setValue(0);

    QLabel *max_val_label = new QLabel(tr("Maximum value ")) ;
    max_val_ = new QSpinBox() ;
    max_val_->setRange(0, 255);
    max_val_->setSingleStep(1);
    max_val_->setValue(255);

    QObject::connect(min_val_, QOverload<int>::of(&QSpinBox::valueChanged), image_canvas_, &ImageCanvas::setMinVal) ;
    QObject::connect(max_val_, QOverload<int>::of(&QSpinBox::valueChanged), image_canvas_, &ImageCanvas::setMaxVal) ;

    QVBoxLayout *min_val_layout = new QVBoxLayout() ;
    min_val_layout->addWidget(min_val_label) ;
    min_val_layout->addWidget(min_val_) ;

    QVBoxLayout *max_val_layout = new QVBoxLayout() ;
    max_val_layout->addWidget(max_val_label) ;
    max_val_layout->addWidget(max_val_) ;

    QHBoxLayout *range_layout = new QHBoxLayout() ;
    range_layout->addLayout(min_val_layout) ;
    range_layout->addLayout(max_val_layout) ;

    return range_layout ;

}

QComboBox *CaptureDashboard::createSensorCombo() {
    QComboBox *combo = new QComboBox() ;

    for( uint i=0 ; i<n_sensors ; i++ )
        combo->addItem(QString("Sensor %1").arg(i+1), i) ;

    return combo ;
}

QHBoxLayout *CaptureDashboard::createSensorCombos() {
    QLabel *sensor_1_label = new QLabel(tr("Topic 1 ")) ;
    sensor_1_ = createSensorCombo() ;

    QVBoxLayout *sensor_1_layout = new QVBoxLayout() ;
    sensor_1_layout->addWidget(sensor_1_label) ;
    sensor_1_layout->addWidget(sensor_1_) ;

    QLabel *sensor_2_label = new QLabel(tr("Topic 2 ")) ;
    sensor_2_ = createSensorCombo() ;

    QVBoxLayout *sensor_2_layout = new QVBoxLayout() ;
    sensor_2_layout->addWidget(sensor_2_label) ;
    sensor_2_layout->addWidget(sensor_2_) ;

    QLabel *sensor_3_label = new QLabel(tr("Topic 3 ")) ;
    sensor_3_ = createSensorCombo() ;

    QVBoxLayout *sensor_3_layout = new QVBoxLayout() ;
    sensor_3_layout->addWidget(sensor_3_label) ;
    sensor_3_layout->addWidget(sensor_3_) ;

    QHBoxLayout *sensor_layout = new QHBoxLayout() ;
    sensor_layout->addLayout(sensor_1_layout) ;
    sensor_layout->addLayout(sensor_2_layout) ;
    sensor_layout->addLayout(sensor_3_layout) ;
    
    return sensor_layout ;

}

void CaptureDashboard::updateImage(uint index, const cv::Mat &im) {
    image_canvas_->updateImage(index, im) ;
}

void CaptureDashboard::setRecordNode(std::shared_ptr<RecordNode> node) {
    record_node_ = node ;
}

int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);

    CaptureDashboard *dashboard = new CaptureDashboard() ;
    auto capture_node = std::make_shared<CaptureNode>(dashboard);
    capture_node->setup() ;
    auto record_node = std::make_shared<RecordNode>(dashboard);
    record_node->setup() ;

    dashboard->setRecordNode(record_node) ;
    
    QMainWindow window ;
    window.setCentralWidget(dashboard) ;
    window.resize(512, 512) ;
    window.show() ;

    
    std::thread t = std::thread([&]{
        rclcpp::executors::MultiThreadedExecutor exec ;
        exec.add_node(capture_node) ;
        exec.add_node(record_node) ;
        exec.spin() ;
        rclcpp::shutdown();
    });


    app.exec();

    t.join();


    return 0;
}