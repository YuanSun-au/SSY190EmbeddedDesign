function  allCombos( west, east )


trainTransition(west, east, 0, 1 )
trainTransition(west, east, 0, 2 )
trainTransition(west, east, 1, 0 )
trainTransition(west, east, 1, 1 )
trainTransition(west, east, 1, 2 )
trainTransition(west, east, 2, 0 )
trainTransition(west, east, 2, 1 )
trainTransition(west, east, 2, 2 )

end

