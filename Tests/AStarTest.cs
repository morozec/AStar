using System;
using System.Collections.Generic;
using AStar;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Tests
{
    [TestClass]
    public class AStarTest
    {
        [TestMethod]
        public void TestPath()
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
                    var square = new Square { X = j, Y = i, Weight = defaultWeight };

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
            Assert.AreEqual(path.Count, 12);
            Assert.AreEqual(path[0], squaresTable[5,0]);
            Assert.AreEqual(path[1], squaresTable[5,1]);
            Assert.AreEqual(path[2], squaresTable[5,2]);
            Assert.AreEqual(path[3], squaresTable[5,3]);
            Assert.AreEqual(path[4], squaresTable[5,4]);
            Assert.AreEqual(path[5], squaresTable[4,4]);
            Assert.AreEqual(path[6], squaresTable[4,5]);
            Assert.AreEqual(path[7], squaresTable[4,6]);
            Assert.AreEqual(path[8], squaresTable[4,7]);
            Assert.AreEqual(path[9], squaresTable[4,8]);
            Assert.AreEqual(path[10], squaresTable[5,8]);
            Assert.AreEqual(path[11], squaresTable[5,9]);
        }
    }
}
