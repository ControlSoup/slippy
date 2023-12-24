pub fn clamp<T>(value: T, max: T, min: T) -> T where T: std::cmp::PartialOrd{
    if value >= max{
        return max
    }
    else if value <= min{
        return min
    }
    else {
        return value
    }
}