#include <UT/UT_Vector.h>
#include <UT/UT_SparseMatrix.h>
#include <hboost/shared_ptr.hpp>

class DiagonalPreconditioner
{
public:
	explicit DiagonalPreconditioner(const UT_SparseMatrix& A) :
		myID(new UT_Vector(0, A.getNumCols() - 1))
	{
		UT_Vector& id(*myID);
		A.extractDiagonal(id);
		// This asserts that A's diagonal is nonzero
		for (int i = 0; i < id.length(); ++i)
			id(i) = 1 / id(i);
	}
	void operator()(const UT_Vector& b, UT_Vector& x) const
	{
		x = b;
		x *= *myID;
	}
private:
	// inverse diagonal
	hboost::shared_ptr<UT_Vector> myID;
};
