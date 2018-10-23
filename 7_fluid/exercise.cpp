//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "gauss_seidel.h"
#include "fluid2d.h"
#include "Utilities/Array2T.h"

// Problem 1
void ExSolvePoisson(int _xRes, int _yRes, int _iterations, double _accuracy, Array2d &_field, Array2d &_b)
{
	double dx = 1.0 / _xRes;
	double residual = _accuracy + 1;
	
	for (int it = 0; residual > _accuracy && it < _iterations; it++)
	{
		// Note that the boundaries are handles by the framework, so you iterations should be similar to:
		for (int y = 1; y < _yRes - 1; ++y)
		{
			for (int x = 1; x < _xRes - 1; ++x)
			{
				// TODO: update the pressure values
				_field(x, y) = 0;
			}
		}

		// Compute the new residual, i.e. the sum of the squares of the individual residuals (squared L2-norm)
		residual = 0;		
		for (int y = 1; y < _yRes - 1; ++y)
		{
			for (int x = 1; x < _xRes - 1; ++x)
			{
				// TODO: compute the cell residual
				double cellResidual = 0;

				residual += cellResidual * cellResidual;
			}
		}

		// Get the L2-norm of the residual
		residual = sqrt(residual);

		// We assume the accuracy is meant for the average L2-norm per grid cell
		residual /= (_xRes - 2) * (_yRes - 2);

		// For your debugging, and ours, please add these prints after every iteration
		if (it == _iterations - 1)
			printf("Pressure solver: it=%d , res=%f \n", it, residual);
		if (residual < _accuracy)
			printf("Pressure solver: it=%d , res=%f, converged \n", it, residual);
	}
}

// Problem 2
void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity)
{
	double dx = 1.0 / _xRes;

	// Note: velocity u_{i+1/2} is practically stored at i+1, hence xV_{i}  -= dt * (p_{i} - p_{i-1}) / dx
	for (int y = 1; y < _yRes - 1; ++y)
		for (int x = 1; x < _xRes; ++x)
			// TODO: update _xVelocity
			_xVelocity(x, y) = 0;
	
	// Same for velocity v_{i+1/2}.
	for (int y = 1; y < _yRes; ++y)
		for (int x = 1; x < _xRes - 1; ++x)
			// TODO: update _yVelocity
			_yVelocity(x, y) = 0;
}

// Problem 3
void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp)
{
	// Densities live on the grid centers, the velocities on the MAC grid
	// Separate their computation to avoid confusion

	// Densities, grid centers
	for (int y = 1; y < _yRes - 1; y++)
	{
		for (int x = 1; x < _xRes - 1; x++)
		{
			// TODO: Compute the velocity
			double last_x_velocity = 0;
			double last_y_velocity = 0;

			// TODO: Find the last position of the particle (in grid coordinates)
			double last_x = 0;
			double last_y = 0;

			// Make sure the coordinates are inside the boundaries
			// Densities are known between 1 and res-2
			if (last_x < 1) last_x = 1;
			if (last_y < 1) last_y = 1;
			if (last_x > _xRes - 2) last_x = _xRes - 2;
			if (last_y > _yRes - 2) last_y = _yRes - 2;

			// Determine corners for bilinear interpolation
			int x_low = (int)last_x;
			int y_low = (int)last_y;
			int x_high = x_low + 1;
			int y_high = y_low + 1;

			// Compute the interpolation weights
			double x_weight = last_x - x_low;
			double y_weight = last_y - y_low;

			// TODO: Bilinear interpolation
			_densityTemp(x, y) = _density(x, y);
		}
	}

	// Velocities (u), MAC grid
	for (int y = 1; y < _yRes - 1; y++)
	{
		for (int x = 1; x < _xRes; x++)
		{
			// TODO: Compute the velocity
			double last_x_velocity = 0;
			double last_y_velocity = 0;

			// TODO: Find the last position of the particle (in grid coordinates)
			double last_x = 0;
			double last_y = 0;

			// Make sure the coordinates are inside the boundaries
			// Being conservative, one can say that the velocities are known between 1.5 and res-2.5
			// (the MAC grid is inside the known densities, which are between 1 and res - 2)
			if (last_x < 1.5) last_x = 1.5;
			if (last_y < 1.5) last_y = 1.5;
			if (last_x > _xRes - 1.5) last_x = _xRes - 1.5;
			if (last_y > _yRes - 2.5) last_y = _yRes - 2.5;

			// Determine corners for bilinear interpolation
			int x_low = (int)last_x;
			int y_low = (int)last_y;
			int x_high = x_low + 1;
			int y_high = y_low + 1;

			// Compute the interpolation weights
			double x_weight = last_x - x_low;
			double y_weight = last_y - y_low;

			// TODO: Bilinear interpolation
			_xVelocityTemp(x, y) = 0;
		}
	}

	// Velocities (v), MAC grid
	for (int y = 1; y < _yRes; y++)
	{
		for (int x = 1; x < _xRes - 1; x++)
		{
			// TODO: Compute the velocity
			double last_x_velocity = 0;
			double last_y_velocity = 0;

			// Find the last position of the particle (in grid coordinates)
			double last_x = x - _dt * _xRes * last_x_velocity;
			double last_y = y - _dt * _yRes * last_y_velocity;

			// Make sure the coordinates are inside the boundaries
			// Being conservative, one can say that the velocities are known between 1.5 and res-2.5
			// (the MAC grid is inside the known densities, which are between 1 and res - 2)
			if (last_x < 1.5) last_x = 1.5;
			if (last_y < 1.5) last_y = 1.5;
			if (last_x > _xRes - 2.5) last_x = _xRes - 2.5;
			if (last_y > _yRes - 1.5) last_y = _yRes - 1.5;

			// Determine corners for bilinear interpolation
			double x_low = (int)last_x;
			double y_low = (int)last_y;
			double x_high = x_low + 1;
			double y_high = y_low + 1;

			// Compute the interpolation weights
			double x_weight = last_x - x_low;
			double y_weight = last_y - y_low;

			// Bilinear interpolation
			_yVelocityTemp(x, y) = 0;
		}
	}

	// Copy the values in temp to the original buffers
	for (int y = 1; y < _yRes - 1; y++)
		for (int x = 1; x < _xRes - 1; x++)
			_density(x, y) = _densityTemp(x, y);
	for (int y = 1; y < _yRes - 1; y++)
		for (int x = 1; x < _xRes; x++)
			_xVelocity(x, y) = _xVelocityTemp(x, y);
	for (int y = 1; y < _yRes; y++)
		for (int x = 1; x < _xRes - 1; x++)
			_yVelocity(x, y) = _yVelocityTemp(x, y);
}