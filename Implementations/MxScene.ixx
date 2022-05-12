export module MxScene;

import <iostream>;

import IMxDebugHelper;

import MxBodyPool;
import MxSolver;

using namespace std;

export struct MxScene {

	MxScene() = delete;
	MxScene(const MxScene& _) = delete;

	/*MxScene(unique_ptr<IMxLogger>&& logger) :
		_solver(move(solver))
	{}*/

	void Fart() {
		cout << "fart\n";
	}

private:

	MxSolver _solver;
	MxBodyPool _bodyPool;
		


};
