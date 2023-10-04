#include <rw/math/Vector2D.hpp>

#include <QApplication>
#include <QChartView>
#include <QLineSeries>
#include <QMainWindow>
#include <vector>

using namespace rw::math;

Vector2D< double > CubicSpline (double t, double ts, double tf, Vector2D< double > Ps,
                                Vector2D< double > Pf, Vector2D< double > Vs, Vector2D< double > Vf)
{
    Vector2D< double > p1 = -2 * (Pf - Ps) + (tf - ts) * (Vs + Vf);
    Vector2D< double > p2 = 3 * (Pf - Ps) - (tf - ts) * (2 * Vs + Vf);
    Vector2D< double > p3 = Vs * (t - ts) + Ps;

    double T = (t - ts) / (tf - ts);

    return p1 * std::pow (T, 3) + p2 * std::pow (T, 2) + p3;
}

int main (int argc, char** argv)
{
    QApplication a (argc, argv);

    Vector2D< double > Vs (0, 1);
    Vector2D< double > Vf (1, 0);
    Vector2D< double > Ps (0, 0);
    Vector2D< double > Pf (1, 2);

    std::vector< double > x, y;
    QLineSeries* series1 = new QLineSeries ();
    QLineSeries* series2 = new QLineSeries ();
    QLineSeries* series3 = new QLineSeries ();

    for (double t = 0; t < 1; t += 0.001) {
        Vector2D< double > p = CubicSpline (t, 0, 1, Ps, Pf, Vs, Vf);
        series1->append (p[0], p[1]);

        p = CubicSpline (t, 0, 1, Ps, Pf, Vs * 2, Vf * 2);
        series2->append (p[0], p[1]);

        p = CubicSpline (t, 0, 1, Ps, Pf, Vs * 6, Vf * 6);
        series3->append (p[0], p[1]);
    }

    QChart* chart = new QChart ();
    chart->legend ()->hide ();
    chart->addSeries (series1);
    chart->addSeries (series2);
    chart->addSeries (series3);
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