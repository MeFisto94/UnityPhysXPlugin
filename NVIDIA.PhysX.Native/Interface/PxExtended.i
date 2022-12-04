typedef	double	PxExtended;
#define	PX_MAX_EXTENDED	PX_MAX_F64
#define PxExtendedAbs(x)	fabs(x)

WRAPPER_STRUCT(PxExtendedVec3)
struct PxExtendedVec3
{
		PxExtendedVec3()																	{}
		PxExtendedVec3(PxExtended _x, PxExtended _y, PxExtended _z) : x(_x), y(_y), z(_z)	{}

	bool isZero()	const
	{
		if(x!=0.0 || y!=0.0 || z!=0.0)	return false;
		return true;
	}

	 PxExtended	dot(const PxVec3& v) const
	{
		return x * PxExtended(v.x) + y * PxExtended(v.y) + z * PxExtended(v.z);
	}

		PxExtended distanceSquared(const PxExtendedVec3& v) const
	{
		PxExtended dx = x - v.x;
		PxExtended dy = y - v.y;
		PxExtended dz = z - v.z;
		return dx * dx + dy * dy + dz * dz;
	}

	 PxExtended magnitudeSquared() const
	{
		return x * x + y * y + z * z;
	}

	 PxExtended magnitude() const
	{
		return PxSqrt(x * x + y * y + z * z);
	}

		PxExtended	normalize()
	{
		PxExtended m = magnitude();
		if (m != 0.0)
		{
			const PxExtended il =  PxExtended(1.0) / m;
			x *= il;
			y *= il;
			z *= il;
		}
		return m;
	}

		bool isFinite()	const
	{
		return PxIsFinite(x) && PxIsFinite(y) && PxIsFinite(z);
	}

		void maximum(const PxExtendedVec3& v)
	{
		if (x < v.x) x = v.x;
		if (y < v.y) y = v.y;
		if (z < v.z) z = v.z;
	}


		void minimum(const PxExtendedVec3& v)
	{
		if (x > v.x) x = v.x;
		if (y > v.y) y = v.y;
		if (z > v.z) z = v.z;
	}

		void	set(PxExtended x_, PxExtended y_, PxExtended z_)
	{
		this->x = x_;
		this->y = y_;
		this->z = z_;
	}

	 void	setPlusInfinity()
	{
		x = y = z = PX_MAX_EXTENDED;
	}

	 void	setMinusInfinity()
	{
		x = y = z = -PX_MAX_EXTENDED;
	}

	 void	cross(const PxExtendedVec3& left, const PxVec3& right)
	{
		// temps needed in case left or right is this.
		PxExtended a = (left.y * PxExtended(right.z)) - (left.z * PxExtended(right.y));
		PxExtended b = (left.z * PxExtended(right.x)) - (left.x * PxExtended(right.z));
		PxExtended c = (left.x * PxExtended(right.y)) - (left.y * PxExtended(right.x));

		x = a;
		y = b;
		z = c;
	}

	 void	cross(const PxExtendedVec3& left, const PxExtendedVec3& right)
	{
		// temps needed in case left or right is this.
		PxExtended a = (left.y * right.z) - (left.z * right.y);
		PxExtended b = (left.z * right.x) - (left.x * right.z);
		PxExtended c = (left.x * right.y) - (left.y * right.x);

		x = a;
		y = b;
		z = c;
	}

	 PxExtendedVec3 cross(const PxExtendedVec3& v) const
	{
		PxExtendedVec3 temp;
		temp.cross(*this,v);
		return temp;
	}

	 void	cross(const PxVec3& left, const PxExtendedVec3& right)
	{
		// temps needed in case left or right is this.
		PxExtended a = (PxExtended(left.y) * right.z) - (PxExtended(left.z) * right.y);
		PxExtended b = (PxExtended(left.z) * right.x) - (PxExtended(left.x) * right.z);
		PxExtended c = (PxExtended(left.x) * right.y) - (PxExtended(left.y) * right.x);

		x = a;
		y = b;
		z = c;
	}

		PxExtendedVec3		operator-()		const
	{
		return PxExtendedVec3(-x, -y, -z);
	}

		PxExtendedVec3&		operator+=(const PxExtendedVec3& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

		PxExtendedVec3&		operator-=(const PxExtendedVec3& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

		PxExtendedVec3&		operator+=(const PxVec3& v)
	{
		x += PxExtended(v.x);
		y += PxExtended(v.y);
		z += PxExtended(v.z);
		return *this;
	}

		PxExtendedVec3&		operator-=(const PxVec3& v)
	{
		x -= PxExtended(v.x);
		y -= PxExtended(v.y);
		z -= PxExtended(v.z);
		return *this;
	}

		PxExtendedVec3&		operator*=(const PxReal& s)
	{
		x *= PxExtended(s);
		y *= PxExtended(s);
		z *= PxExtended(s);
		return *this;
	}

		PxExtendedVec3		operator+(const PxExtendedVec3& v)	const
	{
		return PxExtendedVec3(x + v.x, y + v.y, z + v.z);
	}

		PxVec3			operator-(const PxExtendedVec3& v)	const
	{
		return PxVec3(PxReal(x - v.x), PxReal(y - v.y), PxReal(z - v.z));
	}

	// 	PxExtended&			operator[](int index)
	// {
	// 	PX_ASSERT(index>=0 && index<=2);

	// 	return reinterpret_cast<PxExtended*>(this)[index];
	// }


    %rename(get) operator[];
    PxExtended			operator[](int index) const
	{
		PX_ASSERT(index>=0 && index<=2);

		return reinterpret_cast<const PxExtended*>(this)[index];
	}

	PxExtended x,y,z;
};

PxVec3 toVec3(const PxExtendedVec3& v)
{
    return PxVec3(float(v.x), float(v.y), float(v.z));
}
