#include <stdlib.h> 
#include <iostream>
#include <time.h>

using namespace std;

int userInput(int &numCards)
{
	cout<<"Enter the number of cards you want to practice (must be between 1 and 144):";
	while (true)
	{
		cin>>numCards;	
		// return error if out of bound
		if(numCards<1 || numCards>144)
			cout<<"The number of cards selected is out of range. Please enter a number between 1 and 144:";
		else
			return 0;
	}
	return 0;
}

int swap(int &a, int &b)
{
	int c = a;
	a = b;
	b = c;
	return 0;
}

int shuffle(int flashcard[144][2])
{
	int n;
	srand ((unsigned int)time(NULL));
	for (int i = 0; i < 144; ++i)
	{
		n = rand()%144;
		// cout<<n<<"   ";
		swap(flashcard[i][0],flashcard[n][0]);
		swap(flashcard[i][1],flashcard[n][1]);
	}
	return 0;
}

int main()
{

	int flashcard[144][2];

	for (int i = 0; i < 12; ++i)
	{
		for (int j = 0; j < 12; ++j)
		{
			flashcard[12*i+j][0] = i+1;
			flashcard[12*i+j][1] = j+1;
		}
	}

	// shuffle
	shuffle(flashcard);
	
	// call user input
	int numCards = 0;
	userInput(numCards);

	// This part is for testing the shuffling of cards
	// for (int i = 0; i < 144; ++i)
	// {
	// 	cout<<flashcard[i][0]<<", "<<flashcard[i][1]<<endl;
	// }

	// Start the Game
	int answer;
	int correctAnswers = 0;
	int product;
	time_t rawtime;
	time (&rawtime);
	for (int i = 0; i < numCards; ++i)
	{
		cout<<"New Card is: "<<flashcard[i][0]<<" * "<<flashcard[i][1]<<" = ";
		cin>>answer;
		product = flashcard[i][0]*flashcard[i][1];
		if(product==answer)
		{
			correctAnswers++;
		}
		else
		{
			cout<<"Your answer is wrong. Correct answer is "<<product<<endl;
		}
	}
	time_t finalTime;
	time ( &finalTime );

	double timeTaken;
	timeTaken = double(finalTime - rawtime);
	cout<<"You answered "<<numCards<<" problems in "<<timeTaken<<" seconds"<<endl;
	double percentage;
	percentage = 1.0*correctAnswers/numCards*100;
	cout<<"You answered "<<correctAnswers<<" problems correctly ("<<percentage<<"%)"<<endl;
}
