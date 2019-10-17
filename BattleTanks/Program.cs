using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using AStar;

namespace BattleTanks
{
    class Program
    {
        static void Main(string[] args)
        {
            const int width = 10;
            const int height = 10;
            const int defaultWeight = 1;
            const int bigWeight = 1000;

            var squares = new List<Square>();
            var squaresTable = new Square[height, width];

            for (var i = 0; i < height; ++i)
            {
                for (var j = 0; j < width; ++j)
                {
                    var square = new Square {X = j, Y = i, Weight = defaultWeight};

                    squares.Add(square);
                    squaresTable[i, j] = square;
                }
            }

            for (var i = 0; i < height; ++i)
            {
                for (var j = 0; j < width; ++j)
                {
                    var square = squaresTable[j, i];
                    var neighbours = new List<Square>();
                    if (i > 0) neighbours.Add(squaresTable[j, i - 1]);//up
                    if (j < width - 1) neighbours.Add(squaresTable[j + 1, i]);//right
                    if (i < height - 1) neighbours.Add(squaresTable[j, i + 1]);//down
                    if (j > 0) neighbours.Add(squaresTable[j - 1, i]);//left

                    square.Neighbours = neighbours;
                }
            }

            squaresTable[5, 5].Weight = bigWeight;

            var path = Calculator.GetPath(squaresTable[5, 0], squaresTable[5, 9], squares);

        }
    }
}
