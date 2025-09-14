/* This file is part of GRAMPC-S - (https://github.com/grampc/grampc-s)
 *
 * GRAMPC-S -- A software framework for stochastic model predictive control (SMPC)
 *
 * Copyright 2024 Daniel Landgraf, Andreas Voelz, Knut Graichen.
 * All rights reserved.
 *
 * GRAMPC-S is distributed under the BSD-3-Clause license, see LICENSE.txt
 *
 */


#include "util/grampc_s_util.hpp"

namespace grampc_s
{
    long long factorial(long long n)
    {
        if (n < 1)
        {
            return 1;
        }

        long long returnVal = 1;
        for (long long i = 1; i <= n; ++i)
        {
            returnVal *= i;
        }
        return returnVal;
    }

    QuadratureRuleConstPtr correspondingQuadratureRule(PolynomialFamily polyFam, typeInt quadratureOrder)
    {
        QuadratureRuleConstPtr out;

        switch (polyFam)
        {
        case PolynomialFamily::HERMITE:
            out = QuadratureRuleConstPtr(new HermiteQuadrature(quadratureOrder));
            break;

        case PolynomialFamily::LEGENDRE:
            out = QuadratureRuleConstPtr(new LegendreQuadrature(quadratureOrder));
            break;

        default:
            std::cerr << "No quadrature rule corresponds to this distribution!" << std::endl;
            break;
        }

        return out;
    }

    PolynomialGeneratorPtr correspondingPolynomialGenerator(PolynomialFamily polyFam, typeInt maxOrder)
    {
        PolynomialGeneratorPtr out;

        switch (polyFam)
        {
        case PolynomialFamily::HERMITE:
            out = PolynomialGeneratorPtr(new HermitePolynomialGenerator(maxOrder));
            break;

        case PolynomialFamily::LEGENDRE:
            out = PolynomialGeneratorPtr(new LegendrePolynomialGenerator(maxOrder));
            break;

        default:
            std::cerr << "No set of orthogonal polynomials corresponds to this distribution!" << std::endl;
            break;
        }
        return out;
    }

    void deriveCholesky(VectorRef out, MatrixConstRef chol, typeInt k, typeInt l)
    {
        typeRNum sum;
        typeInt dim = chol.rows();

        for (typeInt i = 0; i < dim; i++)
        {
            for (typeInt j = 0; j <= i; j++)
            {
                sum = (i == k) * (j == l);
                for (typeInt n = 0; n <= j - 1; n++)
                {
                    sum -= (chol(i, n) * out(n * dim + j) + chol(j, n) * out(n * dim + i));
                }
                if (i > j)
                {
                    out(j * dim + i) = (sum - chol(i, j) * out(j * dim + j)) / chol(j, j);
                }
                else if (i == j)
                {
                    out(j * dim + i) = sum / (2 * chol(j, j));
                }
            }
        }
    }
}
