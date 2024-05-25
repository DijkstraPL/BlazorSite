using BeamStatica.Loads.Interfaces;
using System;

namespace BeamStatica.Loads.ContinousLoads
{
    public class ContinousLoad
    {
        public ILoadPosition StartPosition { get; set; }
        public ILoadPosition EndPosition { get; set; }
        public double Length => EndPosition.Position - StartPosition.Position;
        
        public ContinousLoad(ILoadPosition startPosition, ILoadPosition endPosition)
        {
            StartPosition = startPosition ?? throw new ArgumentNullException(nameof(startPosition));
            EndPosition = endPosition ?? throw new ArgumentNullException(nameof(endPosition));
        }

        public double CalculateShear(double distanceFromLoadStartPosition)
        {
            if (distanceFromLoadStartPosition >= this.Length)
               return CalculateShearForceOutsideLoadLength();
            else
                return CalculateShearForceInsideLoadLength(distanceFromLoadStartPosition);
        }

        public double CalculateBendingMoment(double distanceFromLoadStartPosition)
        {
            if (distanceFromLoadStartPosition >= this.Length)
                return CalculateBendingMomentOutsideLoadLength(distanceFromLoadStartPosition);
            else
                return CalculateBendingMomentInsideLoadLength(distanceFromLoadStartPosition);
        }

        private double CalculateShearForceOutsideLoadLength() 
            => ((this.StartPosition.Value + this.EndPosition.Value) * this.Length) / 2;

        private double CalculateShearForceInsideLoadLength(double distanceFromLoadStartPosition)
        {
            double lineAngle = (this.EndPosition.Value - this.StartPosition.Value) / this.Length;

            return (this.StartPosition.Value + (lineAngle * distanceFromLoadStartPosition
                 + this.StartPosition.Value)) * distanceFromLoadStartPosition / 2;
        }

        private double CalculateBendingMomentOutsideLoadLength(double distanceFromLoadStartPosition)
        {
            double bendingMoment = 0;

            bendingMoment += this.StartPosition.Value *
                this.Length / 2 *
                (distanceFromLoadStartPosition - this.Length * 1 / 3);
            bendingMoment += this.EndPosition.Value *
                this.Length / 2 *
               (distanceFromLoadStartPosition - this.Length * 2 / 3);
            return bendingMoment;
        }

        private double CalculateBendingMomentInsideLoadLength(double distanceFromLoadStartPosition)
        {
            double forceAtX = GetForceAtTheCalculatedPoint(distanceFromLoadStartPosition);
            double bendingMoment = 0;

            bendingMoment += this.StartPosition.Value *
               distanceFromLoadStartPosition / 2 *
               distanceFromLoadStartPosition * 2 / 3;
            bendingMoment += forceAtX *
                distanceFromLoadStartPosition / 2 *
                distanceFromLoadStartPosition * 1 / 3;

            return bendingMoment;

        }

        private double GetForceAtTheCalculatedPoint(double distanceFromLoadStartPosition)
            => (this.EndPosition.Value - this.StartPosition.Value) /
               this.Length *
               distanceFromLoadStartPosition +
               this.StartPosition.Value;
    }
}
