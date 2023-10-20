#pragma once
#include "Common/MathTypes.h"
#include "Common/MathExtras.h"
#include "RigidBody.h"

struct Contact
{
	Contact() = default;

	Contact(const uintptr_t& id, const vec2& offsetShapeA, const vec2& offsetShapeB) // TODO befriend?
		: hashId(id),
		  offsetA(offsetShapeA),
		  offsetB(offsetShapeB),
		  normalAcceleration(0),
		  tangentAcceleration(0)
	{
	}

	// These are set when pushing to vector TODO ctor and encapsulate?
	uintptr_t hashId;
	vec2 offsetA;
	vec2 offsetB;

	// These are updated when pre-stepping
	real normalMass;
	real tangentMass;
	real bounce;
	real bias;

	// These are updated when applying impulses
	real normalAcceleration;
	real tangentAcceleration;
	real cachedBias; // Except this which is also reset when pre-stepping

	// Transform offsets to be relative to the positionals passed (e.g. world to shape local)
	void transformOffsets(const Positional& positionalA, const Positional& positionalB)
	{
		offsetA = offsetA - positionalA.position;
		offsetB = offsetB - positionalB.position;
	}
};

struct Manifold
{
	// TODO pack and align to mem access
	// TODO separate contact info
	unsigned count;
	Contact contacts[2];

public:
	vec2 normal;

	/*const*/ Contact& operator [](const size_t i) /*const*/
	{
		assert(i < 2);
		return contacts[i];
	}

	[[nodiscard]] bool isEmpty() const { return count == 0; }
	[[nodiscard]] bool isFull() const { return count == 2; }
	[[nodiscard]] unsigned size() const { return count; }

	void addWorldSpaceContact(const uintptr_t& hash, const vec2& r1, const vec2& r2)
	{
		assert(!isFull());
		contacts[count] = Contact(hash, r1, r2);
		count++;
	}

	// TODO kinda got too OOPish here :(
	// Transforms all contacts in this manifold and sync with existing contacts from another manifold (e.g. persisting previous contacts)
	void transformAndSyncContacts(const Positional& positionalA, const Positional& positionalB, /*const*/ Manifold& existingManifold)
	{
		for (size_t i = 0; i < count; i++)
		{
			auto& contact = contacts[i];
			contact.transformOffsets(positionalA, positionalB);

			for (size_t j = 0; j < existingManifold.size(); j++)
			{
				const auto& existingContact = existingManifold[j];

				if (contact.hashId == existingContact.hashId)
				{
					//TODO copy impulses thingie
					contact.normalAcceleration = existingContact.normalAcceleration;
					contact.tangentAcceleration = existingContact.tangentAcceleration;
					break;
				}
			}
		}
	}
};
