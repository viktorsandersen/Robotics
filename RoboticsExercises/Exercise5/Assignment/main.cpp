#include <rw/math/Vector2D.hpp>

#include <QApplication>
#include <QChartView>
#include <QLineSeries>
#include <QMainWindow>
#include <vector>

using namespace rw::math;


int main (int argc, char** argv)
{
    QApplication a (argc, argv);

    std::vector< double > x, y;
    QLineSeries* series1 = new QLineSeries ();

    for (double t = 0; t < 1; t += 0.001) {
        Vector2D< double > p (t,t);
        series1->append (p[0], p[1]);
    }

    QChart* chart = new QChart ();
    chart->legend ()->hide ();
    chart->addSeries (series1);
    chart->createDefaultAxes ();
    chart->setTitle ("Plotting");

    QChartView* chartView = new QChartView (chart);
    chartView->setRenderHint (QPainter::Antialiasing);

    QMainWindow window;
    window.setCentralWidget (chartView);
    window.resize (400, 300);
    window.show ();

    return a.exec ();
}