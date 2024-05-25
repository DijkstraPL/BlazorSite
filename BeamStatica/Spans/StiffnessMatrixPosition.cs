namespace BeamStatica._spans
{
    public class StiffnessMatrixPosition
    {
        public short RowNumber { get; }
        public short ColumnNumber { get; }
        public double Value { get; }

        public StiffnessMatrixPosition(double value, short rowNumber, short columnNumber)
        {
            Value = value;
            RowNumber = rowNumber;
            ColumnNumber = columnNumber;
        }
    }
}
