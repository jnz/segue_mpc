#include "mpcgain.h"

void mpcgain(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Ap,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Bp,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Cp,
    const unsigned int Nc,
    const unsigned int Np,
    /*output: */Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Phi,
    /*output: */Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& F)
{
    const int ml = (int)Cp.rows();
    const int nl = (int)Bp.rows();
    const int n_in = (int)Bp.cols();

    Phi.resize(Np*ml, Nc);
    F.resize(Np*ml, nl);
    Phi.setZero();
    F.setZero();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h;
    h.resize(Np*ml, nl);
    h.block(0, 0, ml, nl) = Cp;
    F.block(0, 0, ml, nl) = Cp * Ap;

    int jj = ml;
    for (int kk = 1; kk < Np; kk++)
    {
        h.block(jj, 0, ml, nl) = h.block(jj-ml, 0, ml, nl) * Ap;
        F.block(jj, 0, ml, nl) = F.block(jj-ml, 0, ml, nl) * Ap;
        jj += ml;
    }

    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> v = h * Bp;
    Phi.block(0, 0, Np*ml, 1) = v;
    for (int i = 1; i < Nc; i++)
    {
        Phi.block(i*ml, i, Np*ml - i*ml, 1) = v.block(0, 0, Np*ml - i*ml, 1);
    }
}

void mpcgainEx(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Ap,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Bp,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Cp,
    const unsigned int Nc,
    const unsigned int Np,
    /*output: */Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Phi,
    /*output: */Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& F)
{
    const int ml = (int)Cp.rows();
    const int nl = (int)Bp.rows();
    const int n_in = (int)Bp.cols();

    Phi.resize(Np*ml, Nc);
    F.resize(Np*ml, nl + ml);
    Phi.setZero();
    F.setZero();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_e;
    A_e.resize(nl + ml, nl + ml);
    A_e.setIdentity();
    A_e.block(0, 0, nl, nl) = Ap;
    A_e.block(nl, 0, ml, nl) = Cp * Ap;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_e;
    B_e.resize(nl + ml, n_in);
    B_e.block(0, 0, nl, n_in) = Bp;
    B_e.block(nl, 0, ml, n_in) = Cp * Bp;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C_e;
    C_e.resize(ml, nl + ml);
    C_e.setZero();
    C_e.block(0, nl, ml, ml).setIdentity();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h;
    h.resize(Np*ml, nl + ml);
    h.block(0, 0, ml, nl + ml) = C_e;
    F.block(0, 0, ml, nl + ml) = C_e * A_e;

    int jj = ml;
    for (int kk = 1; kk < Np; kk++)
    {
        h.block(jj, 0, ml, nl + ml) = h.block(jj-ml, 0, ml, nl + ml) * A_e;
        F.block(jj, 0, ml, nl + ml) = F.block(jj-ml, 0, ml, nl + ml) * A_e;
        jj += ml;
    }

    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> v = h * B_e;
    Phi.block(0, 0, Np*ml, 1) = v;
    for (int i = 1; i < Nc; i++)
    {
        Phi.block(i*ml, i, Np*ml - i*ml, 1) = v.block(0, 0, Np*ml - i*ml, 1);
    }
}

