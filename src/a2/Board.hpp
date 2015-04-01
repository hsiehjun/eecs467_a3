#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

class Board
{
public:
	int turnState;
	int board[3][3];
	Board();
	void printBoard();
	int getNextMove(int player);
	int hasWinningMove(int p);
	int hasForkingMove(int p);
	int numChecksAt(int row, int col, int p);
	void reset();
	bool hasWon(int p);
	bool openCorner();
	bool full();
	bool isOnLeftDiag( int r, int c);
	bool isOnRightDiag(int r, int c);
	int toOpponent(int p);
	int toInd(int r, int c);
	int toRow(int i);
	int toCol(int i);
	char toChar(int p);
	char toChar(int r, int c);
};
